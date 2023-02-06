#!/usr/bin/env python2
# ***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# ***************************************************************************/

#
# @author Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import ParamValue
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan


class MavrosOffboardPosctlTest(MavrosTestCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.
    For the test to be successful it needs to reach all setpoints in a certain time.
    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def setUp(self):
        super(MavrosOffboardPosctlTest, self).setUp()

        self.pos = PoseStamped()
        self.vel = TwistStamped()
        self.vel_flag = False
        # self.radius = 1
        self.radius = 0.5
        self.barrier_x = 10
        self.barrier_y = 10
        self.barrier_z = 10
        self.x0 = 0
        self.y0 = 0
        self.z0 = 0

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        self.vel_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        # send setpoints in separate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        self.ar_x = 0
        self.ar_y = 0
        self.ar_flag = False
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)
        self.height_from_rangesensor = 0
        #self.height_sub = rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, self.height_callback)
        self.height_sub = rospy.Subscriber("/laser/scan", LaserScan, self.height_callback)
        self.land_flag = False

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #

    def height_callback(self, data):
        #self.height_from_rangesensor = data.range
        self.height_from_rangesensor = data.ranges[0]
        #rospy.loginfo("Height: {0:0.2f}".format(self.height_from_rangesensor))

    def ar_callback(self, data):
        if data.markers == []:
            rospy.loginfo("no marker detect")
            self.ar_flag = False
        else:
            if data.markers[0].id == 16:
                self.ar_flag = True
                #rospy.loginfo("correct AR id: {0}".format(data.markers[0].id))
                x = data.markers[0].pose.pose.position.x
                y = data.markers[0].pose.pose.position.y
                z = data.markers[0].pose.pose.position.z
                #rospy.loginfo("===========  ARucode position x:{0},y:{1},z:{2}".format(x, y, z))
                #self.ar_x = y - 0.1 # real drone bias
                #self.ar_y = -x
                self.ar_x = -y # simulation
                self.ar_y = -x
                #rospy.loginfo("===========  ARucode revised position x:{0:.2f},y:{1:.2f}".format(self.ar_x, self.ar_y))

    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            if self.vel_flag == True:
                self.vel_pub.publish(self.vel)
            else:
                self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint        self.vel_pub.publish(self.vel)

        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0:.2f} of {1:.2f}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(reached, (
            "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
            format(self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, timeout)))

    def send_vel(self, vx, vy, vz, timeout):
        self.vel.twist.linear.x = vx
        self.vel.twist.linear.y = vy
        self.vel.twist.linear.z = vz
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        self.vel_flag = True
        for i in range(timeout * loop_freq):
            # GPS barrier protection
            current_gps_x = self.local_position.pose.position.x
            current_gps_y = self.local_position.pose.position.y
            current_gps_z = self.local_position.pose.position.z
            if current_gps_x < self.x0 - self.barrier_x or current_gps_x > self.x0 + self.barrier_x:
                rospy.loginfo("initial GPS, {0:.2f},{1:.2f},{2:.2f}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of xGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break
            if current_gps_y < self.y0 - self.barrier_y or current_gps_y > self.y0 + self.barrier_y:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of yGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break
            if current_gps_z > self.barrier_z:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of zGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break
            rospy.loginfo(
                "moving speed at  {0},{1},{2}, time: {3} / {4}, curr GPS: {5},{6},{7}".format(vx, vy, vz, i, timeout,
                                                                                              current_gps_x,
                                                                                              current_gps_y,
                                                                                              current_gps_z))
            rate.sleep()

        # Record current GPS and Switch from Vel_cmd to Position_cm
        self.pos.pose.position.x = self.local_position.pose.position.x
        self.pos.pose.position.y = self.local_position.pose.position.y
        self.pos.pose.position.z = self.local_position.pose.position.z

        self.vel_flag = False
        self.vel.twist.linear.x = 0
        # for i in range(timeout * loop_freq):
        #     rospy.loginfo("moving speed at  {0},{1},{2}, time: {3} / {4}".format(0, vy, vz, i, timeout))
        #     rate.sleep()

        # stay there without speed

    def send_vel_auto_height(self, vx, vy, h, timeout):
        self.vel.twist.linear.x = vx
        self.vel.twist.linear.y = vy
        vz = 0.05  # keep in the air
        self.vel.twist.linear.z = vz
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        self.vel_flag = True
        for i in range(timeout * loop_freq):
            # GPS barrier protection
            current_gps_x = self.local_position.pose.position.x
            current_gps_y = self.local_position.pose.position.y
            current_gps_z = self.local_position.pose.position.z
            if current_gps_x < self.x0 - self.barrier_x or current_gps_x > self.x0 + self.barrier_x:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of xGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break
            if current_gps_y < self.y0 - self.barrier_y or current_gps_y > self.y0 + self.barrier_y:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of yGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break
            if current_gps_z > self.barrier_z:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of zGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break
            if self.height_from_rangesensor > h + 0.05:
                vz = -0.05
                self.vel.twist.linear.z = vz
                rospy.loginfo("height_from_rangesensor too high, fly down")
            elif self.height_from_rangesensor < h - 0.05:
                vz = 0.1
                self.vel.twist.linear.z = vz
                rospy.loginfo("height_from_rangesensor too low, fly upper")
            else:
                vz = 0.05
                self.vel.twist.linear.z = vz
                rospy.loginfo("height_from_rangesensor in the h range, keep same height")

            rospy.loginfo(
                "moving speed at  {0},{1},{2}, time: {3} / {4}, curr GPS: {5},{6},{7}".format(vx, vy, vz, i, timeout,
                                                                                              current_gps_x,
                                                                                              current_gps_y,
                                                                                              current_gps_z))
            rate.sleep()

        # Record current GPS and Switch from Vel_cmd to Position_cm
        self.pos.pose.position.x = self.local_position.pose.position.x
        self.pos.pose.position.y = self.local_position.pose.position.y
        self.pos.pose.position.z = self.local_position.pose.position.z
        self.vel_flag = False
        self.vel.twist.linear.x = 0

    def send_vel_land(self, h_start, h_min_for_land, timeout):
        vx = 0
        vy = 0
        h = h_start  # 0.1
        self.vel.twist.linear.x = vx
        self.vel.twist.linear.y = vy
        vz = 0.05  # keep in the air
        self.vel.twist.linear.z = vz
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        self.vel_flag = True

        for i in range(timeout * loop_freq):

            # GPS barrier protection
            current_gps_x = self.local_position.pose.position.x
            current_gps_y = self.local_position.pose.position.y
            current_gps_z = self.local_position.pose.position.z

            if current_gps_x < self.x0 - self.barrier_x or current_gps_x > self.x0 + self.barrier_x:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of xGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break
            if current_gps_y < self.y0 - self.barrier_y or current_gps_y > self.y0 + self.barrier_y:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of yGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break
            if current_gps_z > self.barrier_z:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of zGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break

            h = h - 0.05  # keep decend   can increase the time stay in the air, if you adjust this interval small than 0.05

            # keep the height
            if self.height_from_rangesensor > h + 0.05:
                vz = -0.02
                rospy.loginfo("height_from_rangesensor too high, fly down")
            elif self.height_from_rangesensor < h - 0.05:
                vz = 0.07
                rospy.loginfo("height_from_rangesensor too low, fly upper")
            else:
                vz = 0.05
                rospy.loginfo("height_from_rangesensor in the h range, keep same height")
            # low enough for land
            if h < h_min_for_land:
                rospy.loginfo("reach h_min_for_land and start to land ")
                break

            # Find ar tag
            if self.ar_flag == True:
                # adjust land X position
                if self.ar_x > 0.03:
                    vx = 0.1
                    rospy.loginfo("ar tag on Front, fly front")
                elif self.ar_x < -0.03:
                    vx = -0.1
                    rospy.loginfo("ar tag on Back, fly Back")
                else:
                    vx = 0

                # adjust land Y position
                if self.ar_y > 0.03:
                    vy = 0.1
                    rospy.loginfo("ar tag on Right, fly Right")
                elif self.ar_y < -0.03:
                    vy = -0.1
                    rospy.loginfo("ar tag on Left, fly Left")
                else:
                    vy = 0
            else:
                # not find ar tag, fly higher
                rospy.loginfo("no find ar tag, stay and keep decend")
                vx = 0
                vy = 0
            # h = h + 0.05

            self.vel.twist.linear.x = vx
            self.vel.twist.linear.y = vy
            self.vel.twist.linear.z = vz

            rospy.loginfo(
                "moving speed at  {0},{1},{2}, time: {3} / {4}, curr GPS: {5},{6},{7}".format(vx, vy, vz, i, timeout,
                                                                                              current_gps_x,
                                                                                              current_gps_y,
                                                                                              current_gps_z))
            rate.sleep()

        # Record current GPS and Switch from Vel_cmd to Position_cm
        self.pos.pose.position.x = self.local_position.pose.position.x
        self.pos.pose.position.y = self.local_position.pose.position.y
        self.pos.pose.position.z = self.local_position.pose.position.z
        self.vel_flag = False
        self.vel.twist.linear.x = 0
        self.vel.twist.linear.y = 0
        self.vel.twist.linear.z = 0

    def move_forward_and_land(self, vx_inital, vy_inital, h_start, h_min_for_land, timeout):
        vx = vx_inital
        vy = vy_inital
        vz = 0.05  # keep in the air
        h = h_start  # 1.5 meter
        time_for_aiming_target = 6  # second

        self.vel.twist.linear.x = vx
        self.vel.twist.linear.y = vy
        self.vel.twist.linear.z = vz

        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)

        self.vel_flag = True  # start to use the velocity control
        for i in range(timeout * loop_freq):

            # GPS barrier protection
            current_gps_x = self.local_position.pose.position.x
            current_gps_y = self.local_position.pose.position.y
            current_gps_z = self.local_position.pose.position.z
            if current_gps_x < self.x0 - self.barrier_x or current_gps_x > self.x0 + self.barrier_x:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of xGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break
            if current_gps_y < self.y0 - self.barrier_y or current_gps_y > self.y0 + self.barrier_y:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of yGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break
            if current_gps_z > self.barrier_z:
                rospy.loginfo("initial GPS, {0},{1},{2}".format(self.x0, self.y0, self.z0))
                rospy.loginfo("out of zGPS barrier, current GPS:  {0},{1},{2}, time: {3} / {4}".format(current_gps_x,
                                                                                                       current_gps_y,
                                                                                                       current_gps_z, i,
                                                                                                       timeout))
                break

            # keep the height

            if self.height_from_rangesensor == float("inf") and h > 0.07:    # h goal is still bigger than the min distance so fly up
                vz = 0.07
                rospy.loginfo("height_from_rangesensor is inf, fly upper")
            elif self.height_from_rangesensor == float("inf") and h <= 0.07:    # h goal is small than the min distance so fly down
                vz = -0.07
                rospy.loginfo("height_from_rangesensor is inf, fly down")
            elif self.height_from_rangesensor > h + 0.05:
                vz = -0.07
                rospy.loginfo("height_from_rangesensor too high, fly down")
            elif self.height_from_rangesensor < h - 0.05:
                vz = 0.07
                rospy.loginfo("height_from_rangesensor too low, fly upper")
            else:
                vz = 0.05
                rospy.loginfo("height_from_rangesensor in the h range, keep same height")

            rospy.loginfo("H_Goal:{0:.2f} H_measured:{1:.2f}".format(h, self.height_from_rangesensor))
            # First find ar tag to set the land_flag true
            if self.ar_flag == True:
                self.land_flag = True  # find the first ar code, start to decend

            # After find the first ar tag, start to adjust position and stop moving forward
            if self.land_flag == True:
                if time_for_aiming_target > 0:  # first time to find ar tag , count 6 seconds to adjust pose and then start to decend
                    time_for_aiming_target = time_for_aiming_target - 1  # hove at 1.5 m and aiming the target
                    rospy.loginfo("First detect tag and stay wait for: {0} second and aiming the target".format(
                        time_for_aiming_target))

                if time_for_aiming_target == 0:
                    h = h - 0.05  # keep decend   can increase the time stay in the air, if you adjust this interval small than 0.05
                    rospy.loginfo("Start to decend")

                # Adjust land XY position, according to ar tag
                if self.ar_flag == True:
                    if self.ar_x > 0.03:
                        vx = 0.1
                        rospy.loginfo("ar tag on Front, fly front")
                    elif self.ar_x < -0.03:
                        vx = -0.1
                        rospy.loginfo("ar tag on Back, fly Back")
                    else:
                        vx = 0
                    if self.ar_y > 0.03:
                        vy = 0.1
                        rospy.loginfo("ar tag on Right, fly Right")
                    elif self.ar_y < -0.03:
                        vy = -0.1
                        rospy.loginfo("ar tag on Left, fly Left")
                    else:
                        vy = 0
                else:
                    # not find ar tag, fly higher
                    rospy.loginfo("no find ar tag, stay and keep decend")
                    vx = 0
                    vy = 0
                    # h = h + 0.05

                # low enough for land
                if self.height_from_rangesensor < h_min_for_land:
                    rospy.loginfo("reach h_min_for_land and start to land ")
                    break


            self.vel.twist.linear.x = vx
            self.vel.twist.linear.y = vy
            self.vel.twist.linear.z = vz

            rospy.loginfo(
                "moving speed at x:{0} y:{1} z:{2}, time: {3}/{4}, curr GPS: {5:.2f},{6:.2f},{7:.2f}".format(vx, vy, vz, i, timeout,
                                                                                              current_gps_x,
                                                                                              current_gps_y,
                                                                                              current_gps_z))
            rate.sleep()



        # Record current GPS and Switch from Vel_cmd to Position_cm
        self.pos.pose.position.x = self.local_position.pose.position.x
        self.pos.pose.position.y = self.local_position.pose.position.y
        self.pos.pose.position.z = self.local_position.pose.position.z
        self.vel_flag = False
        self.vel.twist.linear.x = 0
        self.vel.twist.linear.y = 0
        self.vel.twist.linear.z = 0

    #
    # Test method
    #
    def test_posctl(self):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        # exempting failsafe from lost RC to allow offboard
        rcl_except = ParamValue(1 << 2, 0.0)
        self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")

        ##### Check GPS safty ######

        x0_record = []
        y0_record = []
        rate = rospy.Rate(1)  # 1 Hz

        for count in range(3):
            rate.sleep()  # Sleeps for 1/rate sec
            x0_tmp = self.local_position.pose.position.x
            y0_tmp = self.local_position.pose.position.y
            x0_record.append(x0_tmp)
            y0_record.append(y0_tmp)
            rospy.loginfo("GPS position  | {0} , {1}".format(x0_tmp, y0_tmp))
            rospy.loginfo("==================== Recording Initial GPS Position ====================")

        x0_ave = np.average(x0_record)
        y0_ave = np.average(y0_record)
        x0_var = np.var(x0_record)
        y0_var = np.var(y0_record)
        rospy.loginfo("GPS position AVG | {0} , {1} ".format(x0_ave, y0_ave))
        rospy.loginfo("GPS position VAR | {0} , {1} ".format(x0_var, y0_var))

        if x0_var < 1 and y0_var < 1:
            self.x0 = x0_tmp  # x0_ave
            self.y0 = y0_tmp  # y0_ave
            self.z0 = self.local_position.pose.position.z
            rospy.loginfo("initial GPS point {0}, {1}, {2}".format(self.x0, self.y0, self.z0))
            rospy.loginfo("==================== GPS variance is low, good to fly ====================")
        else:
            rospy.loginfo("GPS variance is too high, Stop and disarm")
            self.set_arm(False, 5)
        ##### Check GPS safty ######

        positions = [[0, 0, 1.5]]

        for update_pos in positions:
            update_pos[0] = update_pos[0] + self.x0
            update_pos[1] = update_pos[1] + self.y0
        # positions = ((0, 0, 1.5), (1, 0, 1.5))

        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 30)

        rospy.loginfo("======================================================arrive first point")
        # vx = 0.2
        # vy = 0
        # vz = 0.05
        # timeout = 10
        # self.send_vel(vx,vy,vz,timeout)

        # vx = 0
        # vy = 0
        # vz = 0.05
        # timeout = 20
        # self.send_vel_auto_height(vx,vy,1,timeout)

        # traget land
        # h_start = 1.5
        # h_min_for_land = 0.2
        # timeout = 30
        # self.send_vel_land(h_start, h_min_for_land, timeout)

        # move forward and detect target
        vx_inital = 0.5  # move forward until find the target
        vy_inital = 0  # move left is postive
        h_start = 1.2
        h_min_for_land = 0.2
        timeout = 40  # max move distance 0.2*40 = 8 meter
        self.move_forward_and_land(vx_inital, vy_inital, h_start, h_min_for_land, timeout)

        rospy.loginfo("========================================================End velocity control")

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)


if __name__ == '__main__':
    import rostest

    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)