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
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped, Twist
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
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from scipy.spatial.transform import Rotation as R
from tf.transformations import euler_from_quaternion

# from mavros_msgs.msg import Thrust

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
        self.barrier_x = 10000#10
        self.barrier_y = 10000#10
        self.barrier_z = 100#10
        self.x0 = 0
        self.y0 = 0
        self.z0 = 0

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=10)  # queue_size=1

        self.vel_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10) # queue_size=1

        self.wamv_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # queue_size=1

        # self.thrust_pub = rospy.Publisher(
        #     '/mavros/setpoint_attitude/thrust', Thrust, queue_size=10) # queue_size=1

        # send setpoints in separate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        self.ar_x = 0
        self.ar_y = 0
        self.ar_z = 0
        self.ar_x_trans = 0
        self.ar_y_trans = 0
        self.ar_z_trans = 0
        self.ar_y_tilted = 0
        self.ar_x_last = 0
        self.ar_y_last = 0
        self.ar_x_last_trans = 0
        self.ar_y_last_trans = 0
        self.last_time_trans = rospy.Time.now()
        self.last_time = rospy.Time.now()  # make sure the object is time
        self.ar_yaw = 0
        self.ar_flag = False
        self.ar_flag_s = False
        self.end_velocity_control = False
        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_callback)
        self.height_from_rangesensor = 0
        self.height_from_lidar = 0

        self.subscriber_count = 0
        self.iris_buffer = [[],[],[],[],[],[],[]] # xyz and quaternion
        self.wamv_buffer = [[],[],[],[],[],[],[]] # xyz and quaternion
        #self.iris_speed = [0,0] # x y
        #self.wamv_speed = [0,0] # x y
        self.wamv_estimate_speed = [0,0]
        self.filter_wamv_speed = [0,0]
        self.wamv_estimate_speed_trans = [0,0]
        self.filter_wamv_speed_trans = [0,0]
        self.speed_buffer = [[],[],[],[],[],[],[],[],[],[],[],[],[],[]]
        # drone x y , boat x y, estimate x y, imu drone speed x y,  filter boat speed x y, estimate trans_ar_tag, trans_ar_tag x y
        self.ar_buffer = [[],[],[],[],[],[]]  # ar x y z global   ar estimate xyz   drone global +
        self.data_buffer_x = []
        self.data_buffer_y = []
        self.stop_append = True
        self.land_flag = False
        self.record_len_flag = True  # record # false stop update
        self.record_empty_len = 0

        # self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.integral_x_prior = 0
        self.integral_y_prior = 0
        self.ex_prior = 0
        self.ey_prior = 0

        self.h_for_small_marker = 0 # 2.5 # 2.5 #1.6 # 1 # 0.5 # 0.4
        self.autoland_flag = False

        self.count_low_height = 0
        self.count_low_lidar = 0
        self.mavros_kill_flag = False
        self.flight_mode = False # True: real flight,  False: simulation
        # https://answers.gazebosim.org//question/22125/how-to-set-a-models-position-using-gazeboset_model_state-service-in-python/
        self.first_see_marker = False
        self.tracking_error_x = []
        self.tracking_error_y = []


        self.first_marker_arrive_center = False
        self.iris_vel_cmd_x = 0
        self.iris_vel_cmd_y = -1.8
        # self.height_sub = rospy.Subscriber("/mavros/distance_sensor/hrlv_ez4_pub", Range, self.height_callback)
        self.height_sub = rospy.Subscriber("/mavros/distance_sensor/lidarlite_pub", Range, self.height_callback)
        # self.height_sub = rospy.Subscriber("/laser/scan", LaserScan, self.height_callback)
        self.vel_sub = rospy.Subscriber("/mavros/local_position/velocity_body",TwistStamped, self.vel_callback)
        self.model_states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

        self.vx_cmd_global = 0
        self.vy_cmd_global = 0
        self.count_switch_global = 0
        self.cur_time_record = 0

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()

    #
    # Helper methods
    #

    # def stop_all_rotors(self):
    #
    #     rate = rospy.Rate(100)  # Publishing rate
    #     zero_throttle = Thrust()
    #     zero_throttle.header.stamp = rospy.Time.now()
    #     zero_throttle.thrust = 0.0 # Assuming a quadcopter with 4 rotors
    #
    #     while not rospy.is_shutdown():
    #         self.thrust_pub.publish(zero_throttle)
    #         rate.sleep()

    def write_data(self, filename, value):
        f = open(filename, "w")
        f.write(str(value))
        f.close()

    def read_data(self, filename):
        f = open(filename, 'r')
        cur_iter = f.read()
        f.close()
        return float(cur_iter)


    def update_pso_iter(self):
        f = open('iter.txt', 'r')
        cur_iter = f.read()
        f.close()
        print("curr iter: " + cur_iter)

        f = open("iter.txt", "w")
        next_iter = int(cur_iter)
        next_iter = next_iter + 1
        f.write(str(next_iter))
        f.close()

    def yaw_to_quaternion(self, yaw):
        w = np.cos(yaw / 2)
        x, y, z = 0, 0, np.sin(yaw / 2)

        # Normalize the quaternion
        length = np.sqrt(w**2 + x**2 + y**2 + z**2)
        w /= length
        x /= length
        y /= length
        z /= length

        return x, y, z, w
    def reset(self):
        # method 2
        rospy.wait_for_service('/gazebo/set_model_state')
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state_msg = ModelState()
        state_msg.model_name = 'iris'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 10
        state_msg.pose.position.z = 1
        yaw_angle = 0 # Your yaw angle in radians  270
        quaternion = self.yaw_to_quaternion(yaw_angle)
        state_msg.pose.orientation.x = quaternion[0]
        state_msg.pose.orientation.y = quaternion[1]
        state_msg.pose.orientation.z = quaternion[2]
        state_msg.pose.orientation.w = quaternion[3]

        w_state_msg = ModelState()
        w_state_msg.model_name = 'wamv'
        w_state_msg.pose.position.x = -3  #6
        w_state_msg.pose.position.y = 2.5 #4
        w_state_msg.pose.position.z = 0.1
        yaw_angle = -0.05 #3.14 # Your yaw angle in radians  180
        quaternion = self.yaw_to_quaternion(yaw_angle)
        w_state_msg.pose.orientation.x = quaternion[0]
        w_state_msg.pose.orientation.y = quaternion[1]
        w_state_msg.pose.orientation.z = quaternion[2]
        w_state_msg.pose.orientation.w = quaternion[3]

        #resp = set_state(state_msg)
        resp = set_state(w_state_msg)

    def write_buffer_to_file(self, filename, buffer_list):
        count = 0
        folder_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/path_velocity_record/'
        for inner_list in buffer_list:
            save_filename = folder_path + filename + str(count) + '.txt'
            with open(save_filename, 'a') as file:
                for data in inner_list:
                    file.write(str(data) + '\n')
            count = count + 1
        #buffer_list = []  # Clear the buffer after writing

    def model_states_callback(self, data):
        self.subscriber_count = self.subscriber_count + 1
        wamv_index = data.name.index('wamv')
        iris_index = data.name.index('iris')

        qx_imu = self.imu_data.orientation.x
        qy_imu = self.imu_data.orientation.y
        qz_imu = self.imu_data.orientation.z
        qw_imu = self.imu_data.orientation.w
        imu_quaternion = (qx_imu, qy_imu, qz_imu, qw_imu)  # Replace with your quaternion values
        imu_roll, imu_pitch, imu_yaw = euler_from_quaternion(imu_quaternion)

        quaternion = (data.pose[iris_index].orientation.x, data.pose[iris_index].orientation.y, data.pose[iris_index].orientation.z, data.pose[iris_index].orientation.w)
        roll, pitch, yaw = euler_from_quaternion(quaternion)  # extrinsitc static  sXYZ
        # rospy.loginfo("imu_R:{:2f}, P:{:2f}, Y:{:2f}".format(imu_roll, imu_pitch, imu_yaw))
        # rospy.loginfo("model_R:{:2f}, P:{:2f}, Y:{:2f}".format(roll, pitch, yaw))
        yaw = 0  # only care about relative movement between drone and mar

        roll, pitch, yaw = imu_roll, imu_pitch, 0  # transfer to IMU
        # Calculate individual rotation matrices
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])

        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])

        # Calculate the combined rotation matrix
        R_total = np.dot(Rx, np.dot(Ry, Rz))

        # Create the observed marker position vector
        #marker_position_observed = np.array([-y, -x, -z])  # rotate matrix
        marker_position_observed = np.array([self.ar_x, self.ar_y, self.ar_z])

        # Transform the marker position to the drone's reference frame
        marker_position_transformed = np.dot(R_total, marker_position_observed)

        self.ar_x_trans = marker_position_transformed[0]
        self.ar_y_trans = marker_position_transformed[1]
        self.ar_z_trans = marker_position_transformed[2]

        #1000 / 50 = 20 Hz
        if self.subscriber_count % 50 == 0 and self.stop_append == False:
            #rospy.loginfo("imu_R:{:2f}, P:{:2f}, Y:{:2f}".format(imu_roll, imu_pitch, imu_yaw))
            #rospy.loginfo("model_R:{:2f}, P:{:2f}, Y:{:2f}".format(roll, pitch, yaw))
            #rospy.loginfo("original___ar:[{:2f},{:2f},{:2f}]".format(self.ar_x,self.ar_y,self.ar_z))
            #rospy.loginfo("model_link_ar:[{:2f},{:2f},{:2f}]".format(self.ar_x_trans,self.ar_y_trans,self.ar_z_trans))
            #rospy.loginfo("iris position: {0}".format( data.pose[iris_index].position))

            #rospy.loginfo("iris vel x: {0} y: {1}".format( data.twist[iris_index].linear.x, data.twist[iris_index].linear.y))
            self.iris_buffer[0].append(data.pose[iris_index].position.x)
            self.iris_buffer[1].append(data.pose[iris_index].position.y)
            self.iris_buffer[2].append(data.pose[iris_index].position.z)
            self.iris_buffer[3].append(data.pose[iris_index].orientation.x)
            self.iris_buffer[4].append(data.pose[iris_index].orientation.y)
            self.iris_buffer[5].append(data.pose[iris_index].orientation.z)
            self.iris_buffer[6].append(data.pose[iris_index].orientation.w)
            #rospy.loginfo("wamv position: {0}".format( data.pose[wamv_index].position))
            #rospy.loginfo("wamv vel x: {0} y: {1}".format( data.twist[wamv_index].linear.x, data.twist[wamv_index].linear.y))
            self.wamv_buffer[0].append(data.pose[wamv_index].position.x + 0.1635)   #  -0.199 iris -0.452   wamv -0.253  for small marker
                                                                                    #  + 0.1635 iris -0.0045  wamv -0.168  for big marker
            self.wamv_buffer[1].append(data.pose[wamv_index].position.y)
            self.wamv_buffer[2].append(data.pose[wamv_index].position.z + 1.3572)   # iris 1.25    wamv -0.106   for small marker
            self.wamv_buffer[3].append(data.pose[wamv_index].orientation.x)
            self.wamv_buffer[4].append(data.pose[wamv_index].orientation.y)
            self.wamv_buffer[5].append(data.pose[wamv_index].orientation.z)
            self.wamv_buffer[6].append(data.pose[wamv_index].orientation.w)

            self.speed_buffer[0].append(data.twist[iris_index].linear.x)
            self.speed_buffer[1].append(data.twist[iris_index].linear.y)
            self.speed_buffer[2].append(data.twist[wamv_index].linear.x)
            self.speed_buffer[3].append(data.twist[wamv_index].linear.y)
            self.speed_buffer[4].append(self.wamv_estimate_speed[0])
            self.speed_buffer[5].append(self.wamv_estimate_speed[1])
            self.speed_buffer[6].append(self.vel_from_imu.twist.linear.x)
            self.speed_buffer[7].append(self.vel_from_imu.twist.linear.y)
            self.speed_buffer[8].append(self.filter_wamv_speed[0])
            self.speed_buffer[9].append(self.filter_wamv_speed[1])
            self.speed_buffer[10].append(self.wamv_estimate_speed_trans[0])
            self.speed_buffer[11].append(self.wamv_estimate_speed_trans[1])
            self.speed_buffer[12].append(self.filter_wamv_speed_trans[0])
            self.speed_buffer[13].append(self.filter_wamv_speed_trans[1])

            self.ar_buffer[0].append(data.pose[iris_index].position.x+self.ar_x)
            self.ar_buffer[1].append(data.pose[iris_index].position.y+self.ar_y)
            self.ar_buffer[2].append(data.pose[iris_index].position.z+self.ar_z)
            self.ar_buffer[3].append(data.pose[iris_index].position.x+self.ar_x_trans)
            self.ar_buffer[4].append(data.pose[iris_index].position.y+self.ar_y_trans)
            self.ar_buffer[5].append(data.pose[iris_index].position.z+self.ar_z_trans)

            if self.first_see_marker == True:

                if self.record_len_flag == True: # before seeing the marker and record the empty len
                    self.record_empty_len = len(self.iris_buffer[0])
                    self.record_len_flag = False

                wamv_x_bias =  data.pose[wamv_index].position.x + 0.1635  # for big marker
                error_x = data.pose[iris_index].position.x - wamv_x_bias
                error_y = data.pose[iris_index].position.y - data.pose[wamv_index].position.y
                self.tracking_error_x.append(error_x)
                self.tracking_error_y.append(error_y)

            # # Define the quaternion representing the rotation
            # qx_imu = self.imu_data.orientation.x
            # qy_imu = self.imu_data.orientation.y
            # qz_imu = self.imu_data.orientation.z
            # qw_imu = self.imu_data.orientation.w
            # rotation_quaternion = np.array([qx_imu, qy_imu, qz_imu, qw_imu])  # Replace with your quaternion values
            # # Normalize the quaternion
            # rotation_quaternion /= np.linalg.norm(rotation_quaternion)
            # # Create a Rotation object from the quaternion
            # rotation = R.from_quat(rotation_quaternion)
            # # Apply the rotation to the point
            # marker_position_observed = [x,y,z]
            # transformed_point_by_R = rotation.apply(marker_position_observed)
            # print("x_after_R:", transformed_point_by_R[0])  # (ground truth)
            # print("y_after_R:", transformed_point_by_R[1])
            # print("z_after_R:", transformed_point_by_R[2])
            # self.ar_y_tilted = -transformed_point_by_R[0]

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
                        # self.ar_yaw = yaw # update yaw
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
                            self.ar_y = -x  # np.array([-y, -x, -z])  # rotate matrix
                            self.ar_z = -z
                        self.ar_yaw = yaw
                    rospy.loginfo("id 16 indx_large = {0},  pos = [ {1:.2f}, {2:.2f} ]  yaw = {3:.2f}".format(id_list.index(id_large), self.ar_x, self.ar_y, self.ar_yaw))


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
                    #Kp = 0.5/1.24
                    Kp = 1
                    vg_x = Kp*vs_x - relative_vel_x_boat
                    vg_y = Kp*vs_y - relative_vel_y_boat
                    rospy.loginfo("drone_vel_measured_from_imu          [ {0:.2f}, {1:.2f} ] ".format(vs_x, vs_y))
                    rospy.loginfo("duration {0:.2f},   estimate_boat_vel   [ {1:.2f}, {2:.2f} ] ".format(duration_sec, relative_vel_x_boat, relative_vel_y_boat))
                    rospy.loginfo("global_boat_vel                      [ {0:.2f}, {1:.2f} ] ".format(vg_x, vg_y))
                    #if duration_sec < 0.05:  # only update when sampling rate is quick
                    self.wamv_estimate_speed = [vg_x, vg_y]
                    if duration_sec > 0.05:   # detect enough time to filter extra high speed
                        if abs(vg_x) < 3 and abs(vg_y) < 3:   # manually decrease extreme value
                            # Parameters for the moving average filter
                            window_size = 3  # Number of samples to average
                            self.data_buffer_x.append(vg_x)
                            self.data_buffer_y.append(vg_y)

                            # Maintain the window size
                            if len(self.data_buffer_x) > window_size:
                                self.data_buffer_x.pop(0)
                                self.data_buffer_y.pop(0)

                            data_len_float = len(self.data_buffer_x)*1.0
                            sum_x = sum(self.data_buffer_x)
                            sum_y = sum(self.data_buffer_y)
                            avg_x = sum_x/data_len_float
                            avg_y = sum_y/data_len_float
                            self.filter_wamv_speed = [avg_x, avg_y]


                    curr_time = rospy.Time.now()
                    duration = curr_time-self.last_time_trans
                    duration_sec = duration.to_sec()
                    relative_vel_x_boat = (self.ar_x_last_trans - self.ar_x_trans)/duration_sec
                    relative_vel_y_boat = (self.ar_y_last_trans - self.ar_y_trans)/duration_sec

                    # record last time position and time
                    self.ar_x_last_trans = self.ar_x_trans
                    self.ar_y_last_trans = self.ar_y_trans
                    self.last_time_trans = rospy.Time.now()
                    vs_x = self.vel_from_imu.twist.linear.x
                    vs_y = self.vel_from_imu.twist.linear.y
                    Kp = 1
                    vg_x = Kp*vs_x - relative_vel_x_boat
                    vg_y = Kp*vs_y - relative_vel_y_boat

                    self.wamv_estimate_speed_trans = [vg_x, vg_y]
                    if duration_sec > 0.05:   # detect enough time to filter extra high speed
                        if abs(vg_x) < 3 and abs(vg_y) < 3:   # manually decrease extreme value
                            self.filter_wamv_speed_trans = [vg_x, vg_y]


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
                        # self.ar_yaw = yaw # update yaw
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
                        self.ar_yaw = yaw
                    rospy.loginfo("id 2  indx_small = {0},  pos = [ {1:.2f}, {2:.2f} ] yaw = {3:.2f}".format(id_list.index(id_small), self.ar_x, self.ar_y, self.ar_yaw))

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

                #rospy.loginfo("send boat speed 2.29 m/s")
                wamv_v = Twist()
                wamv_v.linear.x = 2 #2.29 # 0.16 # 0.5 #2.29 #0.5  # max speed    # 0.1  #0.2=0.25  # 0.18= 0.21
                # if self.cur_time_record > 13:
                #     wamv_v.linear.x = -0.6    #-0.4 # 0.15 = 0.18
                # if self.cur_time_record > 20:
                #     wamv_v.linear.x = 0.8  # 2.29

                # 0.1
                # 0.15 = 0.18
                # 0.16 = 0.19
                # 0.2 = 0.25
                # 0.45 = 1.00
                # 0.5 = 1.13
                # 0.6 = 1.5
                # 0.8 = 2.0
                # 1.0 = 2.27
                # 2.29
                # wamv_v.linear.y = -1.0 # lack of motor H do not work
                # wamv_v.angular.z =  -1.3 #-2.0 too much  -1.0 to less -1.1
                self.wamv_pub.publish(wamv_v)
                #rospy.loginfo("send boat speed v2")
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
        yaw_degrees = 0 #-90  # North    # 0 degree = North   #  -90 degree = south
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


    def vel_ramp_funtion(self, vx, vy, acceleration, tolerance, acc_y, tolerance_y):
        if vx > self.vx_cmd_global + tolerance:
            self.vx_cmd_global = self.vx_cmd_global + acceleration
        elif vx < self.vx_cmd_global - tolerance:
            self.vx_cmd_global = self.vx_cmd_global - acceleration
        else:
            self.vx_cmd_global = vx

        if vy > self.vy_cmd_global + tolerance_y:
            self.vy_cmd_global = self.vy_cmd_global + acc_y
        elif vy < self.vy_cmd_global - tolerance_y:
            self.vy_cmd_global = self.vy_cmd_global - acc_y
        else:
            self.vy_cmd_global = vy


        vx_cmd = self.vx_cmd_global
        vy_cmd = self.vy_cmd_global

        # if vx > self.vel_from_imu.twist.linear.x + tolerance:
        #     vx_cmd = self.vel_from_imu.twist.linear.x + acceleration
        # elif vx < self.vel_from_imu.twist.linear.x - tolerance:
        #     vx_cmd = self.vel_from_imu.twist.linear.x - acceleration
        # else:
        #     vx_cmd = vx
        #
        # if vy > self.vel_from_imu.twist.linear.y + tolerance:
        #     vy_cmd = self.vel_from_imu.twist.linear.y + acceleration
        # elif vy < self.vel_from_imu.twist.linear.y - tolerance:
        #     vy_cmd = self.vel_from_imu.twist.linear.y - acceleration
        # else:
        #     vy_cmd = vy

        return vx_cmd, vy_cmd


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
            gyaw = 180*6.28/360.0  # 270*6.28/360.0 # 4.797 #3.14 simulation
        # curr error
        ex = gx - ar_x
        ey = gy - ar_y
        integral_x = self.integral_x_prior + ex * iter_time
        integral_y = self.integral_y_prior + ey * iter_time
        derivative_x = (ex - self.ex_prior) / iter_time
        derivative_y = (ey - self.ey_prior) / iter_time
        # PID
        K_P = 20 #13 #30.0 # 2.0  # 0.5  # high speed 2.0  # self.read_data('Kp.txt') # 2.0 #0.8 #0.3  #0.4 # 0.8 # 1.56
        # if self.height_from_rangesensor < 3:
        #     K_P = 4.0

        K_I = 5 # 5.0 # 0.3  # 0.1  # high speed 0.3  # self.read_data('Ki.txt') # 0 # 0.121
        # if self.height_from_rangesensor < 3:
        #     K_I = 0.6

        # if self.filter_wamv_speed[0] < 0:   # clear integral
        #     integral_x = 0

        K_D = 0.0  # self.read_data('Kd.txt') # 0 #0.4 # 0.245
        vx = K_P * ex + K_I * integral_x + K_D * derivative_x  # pos forward
        vy = K_P * ey + K_I * integral_y + K_D * derivative_y  # pos to left

        # record and update prior value
        self.integral_x_prior = integral_x
        self.integral_y_prior = integral_y
        self.ex_prior = ex
        self.ey_prior = ey

        # boundary
        max_speed = 20
        if abs(vx) > max_speed:   # 1
            vx = np.sign(vx) * max_speed
        if abs(vy) > max_speed:   # 1
            vy = np.sign(vy) * max_speed
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
            gyaw = 180*6.28/360.0  # 270*6.28/360.0  #  4.797 #3.14 simulation
        # curr error
        ex = gx - ar_x
        ey = gy - ar_y
        integral_x = self.integral_x_prior + ex * iter_time
        integral_y = self.integral_y_prior + ey * iter_time
        derivative_x = (ex - self.ex_prior) / iter_time
        derivative_y = (ey - self.ey_prior) / iter_time
        # PID
        K_P = 0.05 # 1.5 # 4.0 # 0.6  # 0.3
        K_I = 0    # 0.02    # 0.1 # 0.1   # 0.01
        K_D = 0    # 0.02   # 0.02
        vx = K_P * ex + K_I * integral_x + K_D * derivative_x  # pos forward
        vy = K_P * ey + K_I * integral_y + K_D * derivative_y  # pos to left

        # record and update prior value
        self.integral_x_prior = integral_x
        self.integral_y_prior = integral_y
        self.ex_prior = ex
        self.ey_prior = ey

        # boundary
        max_speed = 20
        if abs(vx) > max_speed:    # 1
            vx = np.sign(vx) * max_speed
        if abs(vy) > max_speed:      # 1
            vy = np.sign(vy) * max_speed

        eyaw = gyaw - ar_yaw
        # boundary
        if abs(eyaw) > 0.015:  # 0.06 # 1
            vyaw = np.sign(eyaw) * 0.12 # 0.1
        else:
            vyaw = 0

        return vx, vy, vyaw


    def move_hover_land(self, vx_inital, vy_inital, h_start, h_min_for_land, timeout):
        vx = vx_inital
        vy = vy_inital
        vx_cmd = 0
        vy_cmd = 0
        vz = 0.05  # keep in the air
        vyaw = 0
        h = h_start  # 1.2 meter
        time_for_aiming_target = 6  # second
        loop_freq = 20  # Hz
        rate = rospy.Rate(loop_freq)
        time_for_aiming_target = time_for_aiming_target * loop_freq

        self.vel.twist.linear.x = vx  # drone_boat_simulation
        self.vel.twist.linear.y = vy   # drone_boat_simulation
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
                vz = -0.15 # -0.06 # -0.07
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
                    #decend_val = h_start / (timeout * loop_freq)  = (5-1.25=3.75) / 30 * 20 = 5/600
                    h = h - 0.04
                    #h = h -0.01  #-0.00625 #-0.03  #-0.04 #-0.0017 #- 0.002 # - 0.05  # keep decend   can increase the time stay in the air, if you adjust this interval small than 0.05
                    rospy.loginfo("Start to decend") # hover

                # Adjust land XY position, according to ar tag

                if self.height_from_rangesensor > self.h_for_small_marker:
                    if self.ar_flag == True:
                        self.first_see_marker = True   # start to record the average error, so we can elinimate the error between boat and drone
                        # PID control
                        # vx, vy , vyaw = self.vel_PID_controller(self.ar_x, self.ar_y, self.ar_yaw, loop_freq)
                        # vx, vy , vyaw = self.vel_PID_controller(self.ar_x_trans, self.ar_y_trans, self.ar_yaw, loop_freq)
                        if self.ar_x > 0: # x  low speed - 4.7 action boundary high speed 0
                            self.first_marker_arrive_center = True

                        if self.first_marker_arrive_center == True:
                            vx, vy, vyaw = self.vel_PID_controller(self.ar_x_trans, self.ar_y_trans, self.ar_yaw, loop_freq)  # 0.51
                            self.count_switch_global = self.count_switch_global + 1
                            # vx_cmd, vy_cmd = vx, vy

                            # switch to the PID controller
                            if self.count_switch_global < 120:   # 120  ramp function hold time high speed 120
                                rospy.loginfo("============================================================================ ramp function working ================")
                                acceleration = 2.2 #2.2 #2.0 #1.2                  # 0.0125
                                tolerance = acceleration*5
                                acc_y = 0.1
                                tolerance_y = acc_y*5
                                vx_cmd, vy_cmd = self.vel_ramp_funtion(vx, vy, acceleration, tolerance, acc_y, tolerance_y)   #  high speed acceleration 0.1, tolerance 0.5
                            else:
                                vx_cmd, vy_cmd = vx, vy

                        else:  # first aiming y slowing , after that close, give x 0
                            vx, vy, vyaw = self.vel_PID_controller(0, self.ar_y_trans, self.ar_yaw, loop_freq)
                            #vy_cmd = 0.25*vy
                            acceleration = 0.05                   # 0.0125
                            tolerance = acceleration*5
                            acc_y = 0.1
                            tolerance_y = acc_y*5
                            vx_cmd, vy_cmd = self.vel_ramp_funtion(0, vy, acceleration, tolerance, acc_y, tolerance_y)   #  high speed acceleration 0.1, tolerance 0.5


                        # self.iris_vel_cmd_y = -1.8 + vy
                        #     # if self.ar_y < 0:  # ar_tag front speed up
                        #     #     self.iris_vel_cmd_y = -2.5 #self.iris_vel_cmd_y -0.01  #-3.6
                        #     # else:              # ar_tag back  speed down
                        #     #     self.iris_vel_cmd_y = -2#self.iris_vel_cmd_y +0.01   #-2
                        #     #
                        #     # if self.ar_x < 0:  # ar_tag front speed up
                        #     #     self.iris_vel_cmd_x =  0.2 #self.iris_vel_cmd_y -0.001  #-3.6
                        #     # else:              # ar_tag back  speed down
                        #     #     self.iris_vel_cmd_x = -0.2 # self.iris_vel_cmd_y +0.001   #-2
                        #
                        #     rospy.loginfo("ar_tag_y: {:.2f} iris cmd vel: {:.2f}, {:.2f}".format(self.ar_y, self.iris_vel_cmd_y, vy))
                        # else:
                        #     vx = 0 # please wait for object move to the forward
                        #     #vy = -1.8
                        #     self.iris_vel_cmd_y = -1.8
                        #     rospy.loginfo("================================================wait object move to center swith pose ================================================")

                    else:
                        # not find ar tag, fly higher
                        rospy.loginfo("no find ar tag, stay and keep up")
                        vx = 0
                        vy = 0
                        vx_cmd, vy_cmd = self.vel_ramp_funtion(vx, vy, 0.1, 0.5, 0.1, 0.5)
                        # self.iris_vel_cmd_y = 0
                        # self.iris_vel_cmd_x = 0
                        vyaw = 0
                        #h = h + 0.05

                else:  # fly to a low position, change to tracker small tag
                    if self.ar_flag_s == True:

                        # PID control
                        vx, vy , vyaw= self.s_vel_PID_controller(self.ar_x_trans, self.ar_y_trans, self.ar_yaw, loop_freq)
                        # self.iris_vel_cmd_y = -1.8 + vy
                        # self.iris_vel_cmd_x = -1.8 + vx
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
                        self.mavros_kill_flag = False #True  no to land to keep tracking
                        break # June 4 .2023


            self.vel.twist.linear.x = vx_cmd  # vx # self.iris_vel_cmd_y #  vy   # drone_boat_simulation
            self.vel.twist.linear.y = vy_cmd  # vy # self.iris_vel_cmd_x # -vx   # drone_boat_simulation
            self.vel.twist.linear.z = vz
            self.vel.twist.angular.z = vyaw

            cur_time = i * 1.0 / loop_freq
            self.cur_time_record = cur_time
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

        positions = [[0, -7, 5]]   # move forward 1 meter
        #positions = [[0, -5, 5], [5, -5, 5], [10, -5, 5], [10, 0, 5], [0, 0, 5]]   # move forward 1 meter

        for update_pos in positions:
            update_pos[0] = update_pos[0] + self.x0
            update_pos[1] = update_pos[1] + self.y0
        # positions = ((0, 0, 1.5), (1, 0, 1.5))

        # start to record data
        self.stop_append = False   # false = start to record

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
        h_start = 5    # 5
        h_min_for_land = 1.3 # 0.25 # 0.1
        timeout = 30  # max move distance 0.2*40 = 8 meter
        #self.move_forward_and_land(vx_inital, vy_inital, h_start, h_min_for_land, timeout)
        self.move_hover_land(vx_inital, vy_inital, h_start, h_min_for_land, timeout)
        rospy.loginfo("========================================================End velocity control")
        #self.set_mode("AUTO.LAND", 5) # necessary for disarm?

        if self.mavros_kill_flag == False: #######!!!!!!!!!!!!! True
            rospy.loginfo("================= Please notice will trigger force disarm in five seconds =============== ")
            #rospy.sleep(5)  # Sleeps for 5 sec
            rospy.loginfo("======================================================== Force disarm")
            # connection = mavutil.mavlink_connection('udp:localhost:14540')
            # # connection = mavutil.mavlink_connection('udpin:localhost:14540')   # both work
            # # command = 400
            # # param1 = 0  # 1 to ARM, 0 to DISARM
            # # param2 = 21196  # Custom parameter (set to whatever value you need)
            # msg = connection.mav.command_long_encode(
            #     target_system=1,        # Target system ID
            #     target_component=1,     # Target component ID
            #     command=400,
            #     confirmation=0,
            #     param1=0,
            #     param2=21196,
            #     param3=0,
            #     param4=0,
            #     param5=0,
            #     param6=0,
            #     param7=0
            # )
            # connection.mav.send(msg)
            # #self.stop_all_rotors()
            # rospy.loginfo("========================================================run_mavsafety_kill")
            self.first_see_marker = False # stop append error
            self.stop_append = True # stop saving value to buffer
            self.write_buffer_to_file('iris',self.iris_buffer)
            self.write_buffer_to_file('wamv',self.wamv_buffer)
            self.write_buffer_to_file('speed',self.speed_buffer)
            self.write_buffer_to_file('artag',self.ar_buffer)
            ax = self.record_empty_len*[0]+self.tracking_error_x  # make the len as same as other trajectory
            bx = self.record_empty_len*[0]+self.tracking_error_y
            tracking_error_xy = [ax, bx]
            self.write_buffer_to_file('tracking_error', tracking_error_xy)
            self.update_pso_iter()


            if len(self.tracking_error_x) == 0:
                avg_xy = 100 # it means the drone do not see marker in the whole flight
                avg_err_x = 100
                avg_err_y = 100
                rospy.loginfo("do not find any ar_tag")
            else:
                abs_err_x= [abs(num) for num in self.tracking_error_x]
                abs_err_y= [abs(num) for num in self.tracking_error_y]

                avg_err_x = sum(abs_err_x)/len(abs_err_x)
                avg_err_y = sum(abs_err_y)/len(abs_err_y)

                avg_xy = avg_err_x + avg_err_y

            # if avg_xy == 0:
            #     avg_xy = 100 # it means the drone do not see marker in the whole flight

            rospy.loginfo("drone and tag xy distance: {0:.4f}, avg_x: {1:.2f}, avg_y: {2:.2f}, lastpoint_x:{3:.2f}, lastpoint_y:{4:.2f}".format(avg_xy,avg_err_x,avg_err_y,self.tracking_error_x[-1],self.tracking_error_y[-1]))
            self.write_data('f_result.txt', avg_xy)

            # rospy.loginfo("========================================================Complete record buffer")
            # self.run_mavsafety_kill()   # force landing
        else:
            self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,45, 0)
            self.set_arm(False, 5)
        rospy.loginfo("========================================================Reset world")

        self.pos.pose.position.x =  self.x0 # -0.023829642683267593
        self.pos.pose.position.y =  self.y0 # -0.01962843971947829
        self.pos.pose.position.z = 3
        rospy.sleep(10)  # Sleeps for 1 sec

        rospy.loginfo("========================================================moving to the intial point")

        # reset_world
        #self.reset()

        # rospy.loginfo("========================================================Auto_land and disarm again")
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,45, 0)
        self.set_arm(False, 5)

        rospy.loginfo("========================================================Reset boat")
        self.reset()


if __name__ == '__main__':
    now = datetime.now()
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
    print("date and time=",dt_string)
    import rostest

    # rospy.wait_for_service('/gazebo/reset_world')
    rospy.init_node('test_node', anonymous=True)

    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',
                   MavrosOffboardPosctlTest)
