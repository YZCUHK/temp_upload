#!/usr/bin/env python

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
# import urdf
robot = URDF.from_xml_file("/home/yz/catkin_ws/src/aubo_robot/aubo_description/urdf/aubo_i5.urdf")
tree = kdl_tree_from_urdf_model(robot)
chain = tree.getChain("base_Link", "ee_Link")
#print chain.getNrOfJoints()

# forwawrd kinematics
kdl_kin = KDLKinematics(robot, "base_Link", "ee_Link")

global q
global q_ik
global q_current_position
global last_pose
global pose 
global pub_shoulder
global pub_upperArm
global pub_foreArm
global pub_wrist1
global pub_wrist2
global pub_wrist3

# init node and the controller topics
def initnode():
	# global variable definition
	global q
	global q_ik
	global q_current_position
	global last_pose
	global pose
	global sub_joint 
	global pub_shoulder
	global pub_upperArm
	global pub_foreArm
	global pub_wrist1
	global pub_wrist2
	global pub_wrist3
	rospy.init_node('aubo_joint_commander', anonymous=True)
	pub_shoulder = rospy.Publisher('/aubo_i5/manipulate_position_controller_shoulder/command', Float64, queue_size=10)
	pub_upperArm = rospy.Publisher('/aubo_i5/manipulate_position_controller_upperArm/command', Float64, queue_size=10)
	pub_foreArm = rospy.Publisher('/aubo_i5/manipulate_position_controller_foreArm/command', Float64, queue_size=10)
	pub_wrist1 = rospy.Publisher('/aubo_i5/manipulate_position_controller_wrist1/command', Float64, queue_size=10)
	pub_wrist2 = rospy.Publisher('/aubo_i5/manipulate_position_controller_wrist2/command', Float64, queue_size=10)
	pub_wrist3 = rospy.Publisher('/aubo_i5/manipulate_position_controller_wrist3/command', Float64, queue_size=10)

# move to home position
def aubo_command_home():
	q_home=np.array([0,0.78,1.57,0.78,1.57,0.0])
	q_current_position=np.array([0,0,0,0,0,0])
	print "q_current_position : ",q_current_position
	nums=map(abs, q_home-q_current_position)
	nums.sort()
	L=nums[len(nums)-1]
	N=int(math.ceil(L*100))
	rate = rospy.Rate(50)
	for i in range(0,N):
		pub_shoulder.publish(q_current_position[0]+(q_home[0]-q_current_position[0])*(1-math.cos(3.14/N*i))/2.0)
		pub_upperArm.publish(q_current_position[1]+(q_home[1]-q_current_position[1])*(1-math.cos(3.14/N*i))/2.0)
		pub_foreArm.publish(q_current_position[2]+(q_home[2]-q_current_position[2])*(1-math.cos(3.14/N*i))/2.0)
		pub_wrist1.publish(q_current_position[3]+(q_home[3]-q_current_position[3])*(1-math.cos(3.14/N*i))/2.0)
		pub_wrist2.publish(q_current_position[4]+(q_home[4]-q_current_position[4])*(1-math.cos(3.14/N*i))/2.0)
		pub_wrist3.publish(q_current_position[5]+(q_home[5]-q_current_position[5])*(1-math.cos(3.14/N*i))/2.0)
		rate.sleep()
		i=i+1


# relative position to its 'home position', i is initial and f is final, q is radian angle in joint space
def aubo_command(xi,xf,yi,yf,zi,zf,q0,q1,q2,q3,q4,q5):
	L=math.sqrt(math.pow(xf-xi,2)+math.pow(yf-yi,2)+math.pow(zf-zi,2))
	N=int(math.ceil(L*500)+50)    #This is for pose changing
	rate = rospy.Rate(50) # 
	q_ik = np.array([q0,q1,q2,q3,q4,q5])
	last_pose = kdl_kin.forward(q_ik) # forward kinematics (returns homogeneous 4x4 matrix)
	print "last_pose: "
	print last_pose
#------------------------------------------------------------------------------------------------------
#This is pose transformation
#xx xy xz
#yx yy yz
#zx zy zz
#	xx = math.cos(yaw)*math.cos(pitch)
#	xy = math.cos(yaw)*math.sin(pitch)*math.sin(roll)-math.sin(yaw)*math.cos(roll)
#	xz = math.cos(yaw)*math.sin(pitch)*math.cos(roll)+math.sin(yaw)*math.sin(roll)
#	yx = math.sin(yaw)*math.cos(pitch)
#	yy = math.sin(yaw)*math.sin(pitch)*math.sin(roll)+math.cos(yaw)*math.cos(roll)
#	yz = math.sin(yaw)*math.sin(pitch)*math.cos(roll)-math.cos(yaw)*math.sin(roll)
#	zx = -math.sin(pitch)
#	zy = math.cos(pitch)*math.sin(roll)
#	zz = math.cos(pitch)*math.cos(roll)
#------------------------------------------------------------------------------------------------------
	for i in range(0,N):
		pose=last_pose
		pose[0,3]=last_pose[0,3]+xi+(xf-xi)*(1-math.cos(3.14/N*i))/2.0
		pose[1,3]=last_pose[1,3]+yi+(yf-yi)*(1-math.cos(3.14/N*i))/2.0
		pose[2,3]=last_pose[2,3]+zi+(zf-zi)*(1-math.cos(3.14/N*i))/2.0
		print "pose: "
		print pose
		q_ik = kdl_kin.inverse(pose,q_ik) # inverse kinematics
		print q_ik
		pub_shoulder.publish(q_ik[0])
		pub_upperArm.publish(q_ik[1])
		pub_foreArm.publish(q_ik[2])
		pub_wrist1.publish(q_ik[3])
		pub_wrist2.publish(q_ik[4])
		pub_wrist3.publish(q_ik[5])
		rate.sleep()
		i=i+1
	return q_ik


def main():
	initnode()
	aubo_command_home()
	while not rospy.is_shutdown():
		aubo_command_home()
		#q=aubo_command(0,0,0,0,0,0,0,0,0,0,0,0)
		#q=aubo_command(0,0,0,0,0,-0.4,q[0],q[1],q[2],q[3],q[4],q[5])
		#q=aubo_command(0,0.15,0,-0.15,0.15,0,q[0],q[1],q[2],q[3],q[4],q[5]) #(0,0,0.15)->(0.15,-0.15,0)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
