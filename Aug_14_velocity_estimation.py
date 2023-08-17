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
from datetime import datetime
import subprocess
from gazebo_msgs.msg import ModelStates

# from sensor_msgs.msg import LaserScan
# from px4_msgs.msg import VehicleCommand

# from std_srvs.srv import Empty
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
        self.vel_from_imu = TwistStamped()
        self.vel_flag = False
        # self.radius = 1
        self.radius = 0.5
        self.barrier_x = 1000#10
        self.barrier_y = 1000#10
        self.barrier_z = 100#10
        self.x0 = 0
        self.y0 = 0
        self.z0 = 0

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=10)  # queue_size=1

        self.vel_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10) # queue_size=1
        # send setpoints in separate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        self.ar_x = 0
        self.ar_y = 0
        self.ar_x_last = 0
        self.ar_y_last = 0
        self.last_time = rospy.Time.now()  # make sure the object is time
        self.ar_yaw = 0
        self.ar_flag = False
        self.ar_flag_s = False
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)
        self.height_from_rangesensor = 0
        self.height_from_lidar = 0
        # self.height_sub = rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, self.height_callback)
        self.height_sub = rospy.Subscriber("/mavros/distance_sensor/lidarlite_pub", Range, self.height_callback)
        # self.height_sub = rospy.Subscriber("/laser/scan", LaserScan, self.height_callback)
        self.vel_sub = rospy.Subscriber("/mavros/local_position/velocity_body",TwistStamped, self.vel_callback)
        self.model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        self.subscriber_count = 0
        self.iris_buffer = [[],[],[],[],[],[],[]] # xyz and quaternion
        self.wamv_buffer = [[],[],[],[],[],[],[]] # xyz and quaternion
        self.stop_append = False
        self.land_flag = False

        # self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.integral_x_prior = 0
        self.integral_y_prior = 0
        self.ex_prior = 0
        self.ey_prior = 0

        self.h_for_small_marker = 2.5 #1.6 # 1 # 0.5 # 0.4
        self.autoland_flag = False
        self.end_velocity_control = False
        self.count_low_height = 0
        self.count_low_lidar = 0
        self.mavros_kill_flag = False
        self.flight_mode = False # True: real flight,  False: simulation

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #
    def write_buffer_to_file(self, filename, buffer_list):
        count = 0
        self.stop_append = True
        for inner_list in buffer_list:
            save_filename = filename + str(count) + '.txt'
            with open(save_filename, 'a') as file:
                for data in inner_list:
                    file.write(str(data) + '\n')
            count = count + 1
        #buffer_list = []  # Clear the buffer after writing

    def model_states_callback(self, data):
        self.subscriber_count = self.subscriber_count + 1

        #1000 / 50 = 20 Hz
        if self.subscriber_count % 50 == 0 and self.stop_append == False:
            wamv_index = data.name.index('wamv')
            iris_index = data.name.index('iris')
            rospy.loginfo("iris position: {0}".format( data.pose[iris_index].position))
            self.iris_buffer[0].append(data.pose[iris_index].position.x)
            self.iris_buffer[1].append(data.pose[iris_index].position.y)
            self.iris_buffer[2].append(data.pose[iris_index].position.z)
            self.iris_buffer[3].append(data.pose[iris_index].orientation.x)
            self.iris_buffer[4].append(data.pose[iris_index].orientation.y)
            self.iris_buffer[5].append(data.pose[iris_index].orientation.z)
            self.iris_buffer[6].append(data.pose[iris_index].orientation.w)
            rospy.loginfo("wamv position: {0}".format( data.pose[wamv_index].position))
            self.wamv_buffer[0].append(data.pose[wamv_index].position.x -0.705 )   #  iris -0.452   wamv -0.253
            self.wamv_buffer[1].append(data.pose[wamv_index].position.y)
            self.wamv_buffer[2].append(data.pose[wamv_index].position.z + 1.3572)   # iris 1.25    wamv -0.106
            self.wamv_buffer[3].append(data.pose[wamv_index].orientation.x)
            self.wamv_buffer[4].append(data.pose[wamv_index].orientation.y)
            self.wamv_buffer[5].append(data.pose[wamv_index].orientation.z)
            self.wamv_buffer[6].append(data.pose[wamv_index].orientation.w)

            #rospy.loginfo("gazebo")
            #f = open(iris_local.txt, "a")
            #f.write(str(iris_x)+'   '+str(iris_y)+'   '+str(iris_z)+'   '+str(iris_ori_x)+'   '+str(iris_ori_y)+'   '+str(iris_ori_z)+'   '+str(iris_ori_w)+"\n")
        else:
            pass

    def vel_callback(self,data):
        self.vel_from_imu = data

    def height_callback(self, data):
        #self.height_from_rangesensor = data.range  # real
        self.height_from_rangesensor = self.altitude.relative
        self.height_from_lidar = data.range
        if self.autoland_flag == True:
            rospy.loginfo("lidar: {0:0.2f}".format(self.self.height_from_lidar))
            rospy.loginfo("barmeter: {0:0.2f}".format(self.height_from_rangesensor)) # self.altitude.relative
        # self.height_from_rangesensor = data.ranges[0] # simulation
        # rospy.loginfo("Height: {0:0.2f}".format(self.height_from_rangesensor))

    def ar_callback(self, data):

        self.ar_flag = False
        self.ar_flag_s = False

        if self.end_velocity_control == False:
            if data.markers == []:
                rospy.loginfo("no marker detect")
            else:
                id_list = []
                for i in range(len(data.markers)):
                    id_list.append(data.markers[i].id)
                rospy.loginfo("show all marker id_list {0}".format(id_list))

                id_large = 0
                id_small = 1

                if id_large in id_list:
                    self.ar_flag = True
                    indx_large = id_list.index(id_large)
                    x = data.markers[indx_large].pose.pose.position.x
                    y = data.markers[indx_large].pose.pose.position.y
                    z = data.markers[indx_large].pose.pose.position.z

                    try:
                        qx = data.markers[indx_large].pose.pose.orientation.x
                        qy = data.markers[indx_large].pose.pose.orientation.y
                        qz = data.markers[indx_large].pose.pose.orientation.z
                        qw = data.markers[indx_large].pose.pose.orientation.w
                        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                                         qw * qw + qx * qx - qy * qy - qz * qz)  # -3.14 sim will be 0
                        if self.flight_mode == True:
                            pass
                        else:
                            if yaw < 0:
                                yaw = 6.28 + yaw
                        self.ar_yaw = yaw # update yaw
                    except Exception:
                        yaw = self.ar_yaw # use previous yaw
                        # if self.flight_mode == True:
                        #     yaw = 0  # real-world
                        # else:
                        #     yaw = 3.14


                    if self.height_from_rangesensor >= self.h_for_small_marker:  # tracker big marker
                        if self.flight_mode == True:
                            self.ar_x = y  # real drone bias            # tag on the back side of the drone is negative. vx = 0-(-ar_x) = negative so fly back
                            self.ar_y = x  # tag on the left side of drone is positive.     vy = 0-(-ar_y) = positive so fly left
                        else:
                            self.ar_x = -y  # simulation
                            self.ar_y = -x
                        # self.ar_yaw = yaw
                    rospy.loginfo("id 16 indx_large = {0},  pos = [ {1:.2f}, {2:.2f} ]  yaw = {3:.2f}".format(id_list.index(id_large), x, y, yaw))


                    curr_time = rospy.Time.now()
                    duration = curr_time-self.last_time
                    duration_sec = duration.to_sec()
                    relative_vel_x_boat = (self.ar_x_last - self.ar_x)/duration_sec
                    relative_vel_y_boat = (self.ar_y_last - self.ar_y)/duration_sec

                    # record last time position and time
                    self.ar_x_last = self.ar_x
                    self.ar_y_last = self.ar_y
                    self.last_time = rospy.Time.now()
                    vs_x = self.vel_from_imu.twist.linear.x
                    vs_y = self.vel_from_imu.twist.linear.y
                    Kp = 0.5/1.24
                    vg_x = Kp*vs_x - relative_vel_x_boat
                    vg_y = Kp*vs_y - relative_vel_y_boat
                    rospy.loginfo("drone_vel_measured_from_imu          [ {0:.2f}, {1:.2f} ] ".format(vs_x, vs_y))
                    rospy.loginfo("duration {0:.2f},   estimate_boat_vel   [ {1:.2f}, {2:.2f} ] ".format(duration_sec, relative_vel_x_boat, relative_vel_y_boat))
                    rospy.loginfo("global_boat_vel                      [ {0:.2f}, {1:.2f} ] ".format(vg_x, vg_y))


                if id_small in id_list:
                    self.ar_flag_s = True
                    indx_small = id_list.index(id_small)
                    x = data.markers[indx_small].pose.pose.position.x
                    y = data.markers[indx_small].pose.pose.position.y
                    z = data.markers[indx_small].pose.pose.position.z

                    try:
                        qx = data.markers[indx_large].pose.pose.orientation.x
                        qy = data.markers[indx_large].pose.pose.orientation.y
                        qz = data.markers[indx_large].pose.pose.orientation.z
                        qw = data.markers[indx_large].pose.pose.orientation.w
                        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                                         qw * qw + qx * qx - qy * qy - qz * qz)  # -3.14 sim will be 0
                        if self.flight_mode == True:
                            pass
                        else:
                            if yaw < 0:
                                yaw = 6.28 + yaw
                        self.ar_yaw = yaw # update yaw
                    except Exception:
                        yaw = self.ar_yaw # use previous yaw
                        # if self.flight_mode == True:
                        #     yaw = 0  # real-world
                        # else:
                        #     yaw = 3.14

                    if self.height_from_rangesensor < self.h_for_small_marker:  # tracker big marker
                        if self.flight_mode == True:
                            self.ar_x = y  # real drone bias            # tag on the back side of the drone is negative. vx = 0-(-ar_x) = negative so fly back
                            self.ar_y = x  # tag on the left side of drone is positive.     vy = 0-(-ar_y) = positive so fly left
                        else:
                            self.ar_x = -y  # simulation
                            self.ar_y = -x
                        #self.ar_yaw = yaw
                    rospy.loginfo("id 2  indx_small = {0},  pos = [ {1:.2f}, {2:.2f} ] yaw = {3:.2f}".format(id_list.index(id_small), x, y, yaw))

            # if data.markers[0].id == 16:
            # self.ar_flag = True
            ##rospy.loginfo("correct AR id: {0}".format(data.markers[0].id))
            # x = data.markers[0].pose.pose.position.x
            # y = data.markers[0].pose.pose.position.y
            # z = data.markers[0].pose.pose.position.z
            ##rospy.loginfo("===========  ARucode position x:{0},y:{1},z:{2}".format(x, y, z))
            # self.ar_x = y # real drone bias            # tag on the back side of the drone is negative. vx = 0-(-ar_x) = negative so fly back
            # self.ar_y = x                              # tag on the left side of drone is positive.     vy = 0-(-ar_y) = positive so fly left
            ##self.ar_x = -y # simulation
            ##self.ar_y = x
            # rospy.loginfo("===========  ARucode revised position x:{0:.2f},y:{1:.2f}".format(self.ar_x, self.ar_y))

    def send_pos(self):
        rate = rospy.Rate(20)  # 10 Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            if self.vel_flag == True:
                vx = self.vel.twist.linear.x
                vy = self.vel.twist.linear.y
                vz = self.vel.twist.linear.z
                yaw = self.vel.twist.angular.z
                rospy.loginfo("send speed at x:{0:.3f} y:{1:.3f} z:{2}, yaw:{3}".format(vx,vy,vz,yaw))  # add by 2023-05-29
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
        yaw_degrees = -90  # North    # 0 degree = North   #  -90 degree = south
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

    def run_mavsafety_kill(self):
        try:
            # Run the 'rosrun' command with 'mavros' package and 'mavsafety kill' arguments
            subprocess.run(['rosrun', 'mavros', 'mavsafety', 'kill'], check=True)
        except subprocess.CalledProcessError as e:
            rospy.loginfo("Error running command:", e)


    # def send_vehicle_command(connection, command, param1, param2, target_system=1, target_component=1):
    #     msg = connection.mav.command_long_encode(
    #         target_system=target_system,
    #         target_component=target_component,
    #         command=command,
    #         confirmation=0,
    #         param1=param1,
    #         param2=param2,
    #         param3=0,
    #         param4=0,
    #         param5=0,
    #         param6=0,
    #         param7=0
    #     )
    #     connection.mav.send(msg)


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
                self.count_low_height = self.count_low_height + 1

            if self.count_low_height >= 3:
                rospy.loginfo("enough time to triger lower distance")
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

    def vel_PID_controller(self, ar_x, ar_y, ar_yaw, loop_freq):
        iter_time = 1.0 / loop_freq
        # goal is 0
        ar_x = -ar_x  # tag on the back is negative, 0-negtive = po
        ar_y = -ar_y
        gx = 0
        gy = 0
        if self.flight_mode == True:
            gyaw = 0  # real-world
        else:
            gyaw = 4.797 #3.14 simulation
        # curr error
        ex = gx - ar_x
        ey = gy - ar_y
        integral_x = self.integral_x_prior + ex * iter_time
        integral_y = self.integral_y_prior + ey * iter_time
        derivative_x = (ex - self.ex_prior) / iter_time
        derivative_y = (ey - self.ey_prior) / iter_time
        # PID
        K_P = 2.0 #0.8 #0.3  #0.4 # 0.8 # 1.56
        K_I = 0 # 0.121
        K_D = 0 #0.4 # 0.245
        vx = K_P * ex + K_I * integral_x + K_D * derivative_x  # pos forward
        vy = K_P * ey + K_I * integral_y + K_D * derivative_y  # pos to left

        # record and update prior value
        self.integral_x_prior = integral_x
        self.integral_y_prior = integral_y
        self.ex_prior = ex
        self.ey_prior = ey

        # boundary
        if abs(vx) > 12:   # 1
            vx = np.sign(vx) * 1.2
        if abs(vy) > 12:   # 1
            vy = np.sign(vy) * 1.2
        eyaw = gyaw - ar_yaw
        # boundary
        if abs(eyaw) > 0.06:   # 1
            vyaw = np.sign(eyaw) * 0.12 # 0.1
        else:
            vyaw = 0

        return vx, vy, vyaw

    def s_vel_PID_controller(self, ar_x, ar_y, ar_yaw, loop_freq):
        iter_time = 1.0 / loop_freq
        # goal is 0
        ar_x = -ar_x  # tag on the back is negative, 0-negtive = po
        ar_y = -ar_y
        gx = 0
        gy = 0
        if self.flight_mode == True:
            gyaw = 0  # real-world
        else:
            gyaw = 4.797 #3.14 simulation
        # curr error
        ex = gx - ar_x
        ey = gy - ar_y
        integral_x = self.integral_x_prior + ex * iter_time
        integral_y = self.integral_y_prior + ey * iter_time
        derivative_x = (ex - self.ex_prior) / iter_time
        derivative_y = (ey - self.ey_prior) / iter_time
        # PID
        K_P = 1.5  # 4.0 # 0.6  # 0.3
        K_I = 0.02    # 0.1 # 0.1   # 0.01
        K_D = 0.02   # 0.02
        vx = K_P * ex + K_I * integral_x + K_D * derivative_x  # pos forward
        vy = K_P * ey + K_I * integral_y + K_D * derivative_y  # pos to left

        # record and update prior value
        self.integral_x_prior = integral_x
        self.integral_y_prior = integral_y
        self.ex_prior = ex
        self.ey_prior = ey

        # boundary
        if abs(vx) > 12:    # 1
            vx = np.sign(vx) * 1.2
        if abs(vy) > 12:      # 1
            vy = np.sign(vy) * 1.2

        eyaw = gyaw - ar_yaw
        # boundary
        if abs(eyaw) > 0.015:  # 0.06 # 1
            vyaw = np.sign(eyaw) * 0.12 # 0.1
        else:
            vyaw = 0

        return vx, vy, vyaw

    def move_forward_and_land(self, vx_inital, vy_inital, h_start, h_min_for_land, timeout):
        vx = vx_inital
        vy = vy_inital
        vz = 0.05  # keep in the air
        vyaw = 0   # 0 East
        h = h_start  # 1.2 meter
        time_for_aiming_target = 6  # second
        loop_freq = 20  # Hz
        rate = rospy.Rate(loop_freq)
        time_for_aiming_target = time_for_aiming_target * loop_freq

        self.vel.twist.linear.x = vx
        self.vel.twist.linear.y = vy
        self.vel.twist.linear.z = vz

        self.vel_flag = True  # start to use the velocity control   60 * 20 = 1200 from 2m to 0m 
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

            if self.height_from_rangesensor == float(
                    "inf") and h > 0.07:  # h goal is still bigger than the min distance so fly up
                vz = 0.08  # 0.07
                rospy.loginfo("height_from_rangesensor is inf, fly upper")
            elif self.height_from_rangesensor == float(
                    "inf") and h <= 0.07:  # h goal is small than the min distance so fly down
                vz = -0.08 # -0.07
                rospy.loginfo("height_from_rangesensor is inf, fly down")
            elif self.height_from_rangesensor > h + 0.05:
                vz = -0.08  #-0.07
                rospy.loginfo("height_from_rangesensor too high, fly down")
            elif self.height_from_rangesensor < h - 0.05:
                vz = 0.09  # 0.08  #0.07
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
                    remain_second = 1.0 * time_for_aiming_target / loop_freq
                    rospy.loginfo(
                        "======================= same height and aiming for: {0} the remain {1:.2f} second ".format(
                            time_for_aiming_target, remain_second))

                if time_for_aiming_target == 0:
                    h = h - 0.002 # - 0.05  # keep decend   can increase the time stay in the air, if you adjust this interval small than 0.05
                    rospy.loginfo("Start to decend")

                # Adjust land XY position, according to ar tag

                if self.height_from_rangesensor > self.h_for_small_marker:
                    if self.ar_flag == True:
                        # PID control
                        vx, vy , vyaw = self.vel_PID_controller(self.ar_x, self.ar_y, self.ar_yaw, loop_freq)
                    else:
                        # not find ar tag, fly higher
                        rospy.loginfo("no find ar tag, stay and keep decend")
                        vx = 0
                        vy = 0
                        vyaw = 0
                        # h = h + 0.05

                else:  # fly to a low position, change to tracker small tag
                    if self.ar_flag_s == True:
                        # PID control
                        vx, vy , vyaw= self.s_vel_PID_controller(self.ar_x, self.ar_y, self.ar_yaw, loop_freq)
                        rospy.loginfo("!!!!!!!!!!!!!!!!!!  detect small ar tag now !!!!!!!!!!!!!!!!!!!!!!")
                    else:
                        # not find ar tag, fly higher
                        rospy.loginfo("no find ar tag, stay and keep decend")
                        vx = 0
                        vy = 0
                        vyaw = 0
                        # h = h + 0.05

                # low enough for land
                if self.height_from_rangesensor < h_min_for_land:
                    rospy.loginfo("reach h_min_for_land and start to land ")
                    self.autoland_flag = True
                    break

            self.vel.twist.linear.x = vx
            self.vel.twist.linear.y = vy
            self.vel.twist.linear.z = vz
            self.vel.twist.angular.z = vyaw

            cur_time = i * 1.0 / loop_freq
            rospy.loginfo(
                "moving speed at x:{0:.2f} y:{1:.2f} z:{2}, time: {3}/{4}, curr GPS: {5:.2f},{6:.2f},{7:.2f}, yaw:{8}".format(vx,
                                                                                                                     vy,
                                                                                                                     vz,
                                                                                                                     cur_time,
                                                                                                                     timeout,
                                                                                                                     current_gps_x,
                                                                                                                     current_gps_y,
                                                                                                                     current_gps_z,vyaw))
            rate.sleep()

        # Record current GPS and Switch from Vel_cmd to Position_cm
        self.pos.pose.position.x = self.local_position.pose.position.x
        self.pos.pose.position.y = self.local_position.pose.position.y
        self.pos.pose.position.z = self.local_position.pose.position.z
        self.vel_flag = False
        self.vel.twist.linear.x = 0
        self.vel.twist.linear.y = 0
        self.vel.twist.linear.z = 0
        self.vel.twist.angular.z = 0  # stop rotating


    def move_hover_land(self, vx_inital, vy_inital, h_start, h_min_for_land, timeout):
        vx = vx_inital
        vy = vy_inital
        vz = 0.05  # keep in the air
        vyaw = 0
        h = h_start  # 1.2 meter
        time_for_aiming_target = 6  # second
        loop_freq = 20  # Hz
        rate = rospy.Rate(loop_freq)
        time_for_aiming_target = time_for_aiming_target * loop_freq

        self.vel.twist.linear.x = vy   # drone_boat_simulation
        self.vel.twist.linear.y = -vx   # drone_boat_simulation
        self.vel.twist.linear.z = vz

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

            if self.height_from_rangesensor == float(
                    "inf") and h > 0.07:  # h goal is still bigger than the min distance so fly up
                vz = 1.00 # 0.07
                rospy.loginfo("height_from_rangesensor is inf, fly upper")
            elif self.height_from_rangesensor == float(
                    "inf") and h <= 0.07:  # h goal is small than the min distance so fly down
                vz = -0.1 # -0.06 # -0.07
                rospy.loginfo("height_from_rangesensor is inf, fly down")
            elif self.height_from_rangesensor > h + 0.05:
                vz = -0.1 # -0.06 # -0.07
                rospy.loginfo("height_from_rangesensor too high, fly down")
            elif self.height_from_rangesensor < h - 0.05:
                vz = 1.00 #0.08 # 0.07
                rospy.loginfo("height_from_rangesensor too low, fly upper")
            else:
                vz = 0.07 # 0.05
                rospy.loginfo("height_from_rangesensor in the h range, keep same height")

            rospy.loginfo("H_Goal:{0:.2f} barmeter:{1:.2f} lidar:{2:.2f}".format(h, self.height_from_rangesensor, self.height_from_lidar))
            # First find ar tag to set the land_flag true
            if self.ar_flag == True:
                self.land_flag = True  # find the first ar code, start to decend

            # After find the first ar tag, start to adjust position and stop moving forward
            if self.land_flag == True:
                if time_for_aiming_target > 0:  # first time to find ar tag , count 6 seconds to adjust pose and then start to decend
                    time_for_aiming_target = time_for_aiming_target - 1  # hove at 1.5 m and aiming the target
                    remain_second = 1.0 * time_for_aiming_target / loop_freq
                    rospy.loginfo(
                        "======================= same height and aiming for: {0} the remain {1:.2f} second ".format(
                            time_for_aiming_target, remain_second))

                if time_for_aiming_target == 0:
                    #decend_val = h_start / (timeout * loop_freq)
                    h = h -0.01 #-0.0017 #- 0.002 # - 0.05  # keep decend   can increase the time stay in the air, if you adjust this interval small than 0.05
                    #
                    rospy.loginfo("Start to decend") # hover

                # Adjust land XY position, according to ar tag

                if self.height_from_rangesensor > self.h_for_small_marker:
                    if self.ar_flag == True:
                        # PID control
                        vx, vy , vyaw = self.vel_PID_controller(self.ar_x, self.ar_y, self.ar_yaw, loop_freq)
                    else:
                        # not find ar tag, fly higher
                        rospy.loginfo("no find ar tag, stay and keep up")
                        vx = 0
                        vy = 0
                        vyaw = 0
                        #h = h + 0.05


                else:  # fly to a low position, change to tracker small tag
                    if self.ar_flag_s == True:
                        # PID control
                        vx, vy , vyaw= self.s_vel_PID_controller(self.ar_x, self.ar_y, self.ar_yaw, loop_freq)
                        rospy.loginfo("!!!!!!!!!!!!!!!!!!  detect small ar tag now !!!!!!!!!!!!!!!!!!!!!!")
                    else:
                        # not find ar tag, fly higher
                        rospy.loginfo("no find ar tag, stay and keep up")
                        vx = 0
                        vy = 0
                        vyaw = 0
                        #h = h + 0.05

                # goal_height_max_boundry
                if h > h_start:
                    h = h_start
                if h < 0:
                    h = 0
                # low enough for land
                # if self.height_from_rangesensor < h_min_for_land:
                #     rospy.loginfo("reach h_min_for_land and start to land ")
                #     self.autoland_flag = True
                #     break


                # low enough for land
                if self.height_from_rangesensor < h_min_for_land:  # 1.3
                    self.count_low_height = self.count_low_height + 1
                    rospy.loginfo("reach h_min_for_land and start to land {0} time".format(self.count_low_height))


                if self.count_low_height >= 3:
                    rospy.loginfo("over 3 time barmeter limit, now start to count lidar")

                    if self.height_from_lidar < 0.25:  # lidar min height
                        self.count_low_lidar = self.count_low_lidar + 1
                        rospy.loginfo("reach lidar min height {0} time".format(self.count_low_lidar))

                    if self.count_low_lidar >= 3:
                        rospy.loginfo("over 3 time lidar limit, now to force disarm ")
                        self.mavros_kill_flag = True
                        break # June 4 .2023


            self.vel.twist.linear.x = vy   # drone_boat_simulation
            self.vel.twist.linear.y = -vx   # drone_boat_simulation
            self.vel.twist.linear.z = vz
            self.vel.twist.angular.z = vyaw

            cur_time = i * 1.0 / loop_freq
            rospy.loginfo(
                "moving speed at x:{0:.3f} y:{1:.3f} z:{2}, time: {3}/{4}, curr GPS: {5:.2f},{6:.2f},{7:.2f}, yaw:{8}".format(vx,
                                                                                                                     vy,
                                                                                                                     vz,
                                                                                                                     cur_time,
                                                                                                                     timeout,
                                                                                                                     current_gps_x,
                                                                                                                     current_gps_y,
                                                                                                                     current_gps_z,vyaw))
            rate.sleep()

        # Record current GPS and Switch from Vel_cmd to Position_cm
        self.pos.pose.position.x = self.local_position.pose.position.x
        self.pos.pose.position.y = self.local_position.pose.position.y
        self.pos.pose.position.z = self.local_position.pose.position.z
        self.vel_flag = False
        self.vel.twist.linear.x = 0
        self.vel.twist.linear.y = 0
        self.vel.twist.linear.z = -0.07
        self.vel.twist.angular.z = 0  # stop rotating
        self.end_velocity_control = True # stop publishing marker

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

        positions = [[0, -5, 5]]   # move forward 1 meter

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
        vx_inital = 0  # move forward until find the target
        vy_inital = 0  # move left is postive
        h_start = 5
        h_min_for_land = 1.3 # 0.25 # 0.1
        timeout = 60  # max move distance 0.2*40 = 8 meter
        #self.move_forward_and_land(vx_inital, vy_inital, h_start, h_min_for_land, timeout)
        self.move_hover_land(vx_inital, vy_inital, h_start, h_min_for_land, timeout)
        rospy.loginfo("========================================================End velocity control")
        self.set_mode("AUTO.LAND", 5)

        if self.mavros_kill_flag == True:
            rospy.loginfo("================= Please notice will trigger force disarm in five seconds =============== ")
            #rospy.sleep(5)  # Sleeps for 5 sec
            rospy.loginfo("======================================================== Force disarm")
            connection = mavutil.mavlink_connection('udp:localhost:14540')
            # command = 400
            # param1 = 0  # 1 to ARM, 0 to DISARM
            # param2 = 21196  # Custom parameter (set to whatever value you need)
            msg = connection.mav.command_long_encode(
                target_system=1,        # Target system ID
                target_component=1,     # Target component ID
                command=400,
                confirmation=0,
                param1=0,
                param2=21196,
                param3=0,
                param4=0,
                param5=0,
                param6=0,
                param7=0
            )
            connection.mav.send(msg)

            rospy.loginfo("========================================================run_mavsafety_kill")
            self.write_buffer_to_file('iris',self.iris_buffer)
            self.write_buffer_to_file('wamv',self.wamv_buffer)
            rospy.loginfo("========================================================Complete record buffer")
            self.run_mavsafety_kill()
        else:
            self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,45, 0)
            self.set_arm(False, 5)


        rospy.loginfo("========================================================End re set world")


        rospy.sleep(5)  # Sleeps for 1 sec
        # self.reset_world()


if __name__ == '__main__':
    now = datetime.now()
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
    print("date and time=",dt_string)
    import rostest

    # rospy.wait_for_service('/gazebo/reset_world')
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)
