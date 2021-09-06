#!/usr/bin/env python
# Obtain joint states of the winbot

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import os, time
import sys

def joint_states_obtainer():
	rospy.init_node('joint_states_obtainer', anonymous=True)
	rospy.Subscriber('/aubo_i5/joint_states', JointState, callback)
	rospy.spin()

def callback(data):
	#rate=rospy.Rate(1)
	#rate.sleep()
	print data.name
	print data.position
	print data.velocity



if __name__ == '__main__':
    try:
        joint_states_obtainer()
    except rospy.ROSInterruptException:
        pass
