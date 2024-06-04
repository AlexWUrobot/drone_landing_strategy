
PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import ParamValue
from mavros_test_common import MavrosTestCommon
from mavros_msgs.msg import ActuatorControl
#from mavros_msgs.msg import CommandBool, SetMode


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
        self.radius = 0.5

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def tearDown(self):
        super(MavrosOffboardPosctlTest, self).tearDown()



def send_servo_command():
    pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)
    rate = rospy.Rate(10)  # Set the publishing rate (e.g., 10 Hz)

    while not rospy.is_shutdown():
        msg = ActuatorControl()
        msg.header.stamp = rospy.Time.now()
        msg.group_mix = 0  # Group 0 for servos
        msg.controls = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Set servo values here
        pub.publish(msg)
        rate.sleep()
    #
    # Test method
    #
    #def test_posctl(self):
        #"""Test offboard position control"""
    	#rospy.loginfo("run servo-------------------------------------------~~~~~")



if __name__ == '__main__':
    import rostest
    rospy.init_node('test_node', anonymous=True)
    rostest.rosrun(PKG, 'mavros_offboard_posctl_test',MavrosOffboardPosctlTest)
    try:
    	# Initialize the offboard mode
    	#set_mode_pub = rospy.Publisher('/mavros/set_mode', SetMode, queue_size=1)
    	#offboard_mode = SetMode()
    	#offboard_mode.custom_mode = 'OFFBOARD'
    	#set_mode_pub.publish(offboard_mode)

   	# Arm the vehicle (if needed)
    	#arm_pub = rospy.Publisher('/mavros/cmd/arming', CommandBool, queue_size=1)
    	#arm_cmd = CommandBool()
    	#arm_cmd.value = True
    	#arm_pub.publish(arm_cmd)

	rospy.loginfo("run servo------------------------------------------")
    	# Continue with your servo control logic
        send_servo_command()

    except rospy.ROSInterruptException:
        pass
