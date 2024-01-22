#!/usr/bin/env python
# coding: UTF-8

import sys
from math import pi

import moveit_commander
import rospy

import numpy as np

# import datetime
# import time

import actionlib
from moveit_tutorials.msg import EmptyAction, EmptyGoal, EmptyResult
import csv

# dataset = 400

def MovetoGoal(goal):
	# joint_goal = [1.613207, -1.609717, 1.879719, -0.252549, 1.326974, -3.155381] ###desired1
	# joint_goal = [1.60414, -1.90515, 2.10868, -0.18457, 1.31903, -3.15687] ###desired2
	# joint_goal = [2.088053, -1.572962, 1.842943, -0.251741, 1.802664,	-3.147260] ###desired3
	
	# joint_goal_deg = [92.43, -92.23, 107.7, -14.47, 76.03, -180.79] ###desired1
	# joint_goal_deg = [89.06, -75.86, 90.32, -14.37, 88.57, -0.09] ###deisred 1 v2
	joint_goal_deg = [89.07, -75.87, 90.32, -14.37, 88.57, -0.09] ###desired1 v3
	# joint_goal_deg = [90.92, -90.08, 106.67, -16.50, 90.40, -0.22] ### desired 2 v2
	# joint_goal_deg = [119.63, -90.13, 105.60, -14.43, 103.28, -180.32] ### desired 3
 
	joint_goal = [x * pi/180 for x in joint_goal_deg]

	move_group.set_joint_value_target(joint_goal)
	move_group.go(wait=True)
	print('moved to desires pose')
 
	theta = []
	pose_data = [] ###リストのサイズ指定
	current_joint_values = move_group.get_current_joint_values()
	theta.append(current_joint_values)
	
	posex = move_group.get_current_pose().pose.position.x
	posey = move_group.get_current_pose().pose.position.y
	posez = move_group.get_current_pose().pose.position.z
	pose_data.append([posex, posey, posez])
	filename1 = './valid_joint_values/desired1_v3_25/desired_theta_result.csv'
	filename2 = './valid_joint_values/desired1_v3_25/desired_pose.csv'
	with open (filename1, 'a') as f1, open (filename2, 'a') as f2:
		writer = csv.writer(f1)
		writer.writerows(theta)
		writer2 = csv.writer(f2)
		writer2.writerows(pose_data)
	result = EmptyResult()
	motion_server.set_succeeded(result)
	
if __name__ == "__main__":
	rospy.init_node('motion_server')
	moveit_commander.roscpp_initialize(sys.argv)
	move_group = moveit_commander.MoveGroupCommander('manipulator')
	move_group.set_max_velocity_scaling_factor(value = 0.08)
	move_group.set_max_acceleration_scaling_factor(value = 0.08)
	rospy.loginfo('motion server started. ready to serve.')
	motion_server = actionlib.SimpleActionServer('ugoke', EmptyAction, MovetoGoal, False)
	motion_server.start()
	rospy.spin()