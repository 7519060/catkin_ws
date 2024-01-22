#!/usr/bin/env python
# coding: UTF-8

import sys
import moveit_commander
import rospy
from math import pi

def Move():
	moveit_commander.roscpp_initialize(sys.argv)
	move_group = moveit_commander.MoveGroupCommander("manipulator")
	move_group.set_max_velocity_scaling_factor(value=0.02)
	move_group.set_max_acceleration_scaling_factor(value=0.02)
	# move_group.set_joint_value_target(joint_goal)
	# print(joint_goal) ###type list deshita
	# move_group.go(wait=True)
	# print('moved to waypoint 1')
	
	joint_goal2_deg = [89.07, -75.87, 90.32, -14.37, 88.57, -0.09] ###ddesireed 1 v3
	# joint_goal2_deg = [90.92, -90.08, 106.67, -16.50, 90.40, -0.22] ###desired 2_v2
	joint_goal2 = [x * pi/180 for x in joint_goal2_deg]

	move_group.set_joint_value_target(joint_goal2)
	print(joint_goal2)
	move_group.go(wait=True)
	print('moved to desired pose')
 
	moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
	rospy.init_node('desired_pose')
	Move()
 
 
 ####あとでdsrth_resultのdesired_joint.csvとかからジョイント角決定してjoint_goal2_degとする
 ### 出てきたジョイント角rad入れて実機動かしたときのペンダントの値を採用？