#!/usr/bin/env python
# coding: UTF-8

import sys
from math import pi

import moveit_commander
import rospy

import random
import numpy as np

# import datetime
# import time

from time import sleep

import csv

import pandas as pd

# def MovetoGoal():
# 	theta = []
# 	now = datetime.datetime.now()
# 	filename1 = './' + now.strftime('%Y%m%d_%H%M%S') + '_joint_goal_result.txt'
# 	# filename1 = './resulttt.txt'
# 	with open (filename1, "a") as f:
# 		####go to mokuhyo pose####
# 		# initialize MoveitCommander
# 		moveit_commander.roscpp_initialize(sys.argv)

# 		# prepare MoveGroupCommander
# 		move_group = moveit_commander.MoveGroupCommander("manipulator")

# 		# set max velocity
# 		move_group.set_max_velocity_scaling_factor(value=0.05)
# 		move_group.set_max_acceleration_scaling_factor(value=0.05)

# 		# set joint to define goal state
# 		joint_goal = [1.5708, -0.9225, 0.5934, -1.2217, -1.5708, 0.0]
# 		move_group.set_joint_value_target(joint_goal)

# 		# plan motion plan and execute
# 		move_group.go(wait=True)

# 		#stop
# 		move_group.stop()

# 		print('moved to mokuhyo pose')

# 		theta.append(joint_goal)

# 		print(theta)
# 		f.write(str(theta))

# def Move():
# 	theta = []
# 	moveit_commander.roscpp_initialize(sys.argv)

# 	# prepare MoveGroupCommander
# 	move_group = moveit_commander.MoveGroupCommander("manipulator")

# 	# set max velocity
# 	move_group.set_max_velocity_scaling_factor(value=0.05)
# 	move_group.set_max_acceleration_scaling_factor(value=0.05)

# 	# set joint to define goal state
# 	joint_goal = np.array([1.5708, -0.9225, 0.5934, -1.2217, -1.5708, 0.0])
# 	# print(type(joint_goal))
# 	# print(joint_goal.shape)
# 	# test_joint_goal = np.empty((6))
# 	# test_joint_goal = joint_goal[:,0]
# 	# print(type(test_joint_goal))
# 	# print(test_joint_goal.shape)
# 	# move_group.set_joint_value_target(joint_goal)
# 	move_group.set_joint_value_target(joint_goal)

# 	# plan motion plan and execute
# 	move_group.go(wait=True)

# 	#stop
# 	move_group.stop()

# 	print('moved to mokuhyo pose')

# 	theta.append(joint_goal)

# 	print(theta)
  	

# def MovetoRandomGoal():
# 	theta=[]
# 	# pose_data = []
# 	# now = datetime.datetime.now()
# 	# filename2 = './testresult/' + now.strftime('%Y%m%d_%H%M%S') + '_test_result.csv'
# 	filename2 = './test_result.csv'
# 	with open (filename2, "a") as f:
# 		for i in range(3):
# 			moveit_commander.roscpp_initialize(sys.argv)
			
# 			move_group = moveit_commander.MoveGroupCommander("manipulator")
# 			#多少は横にも動くようになった
# 			joint_goal = [random.uniform(1.55, 1.59), random.uniform(-1.24, -1.20), random.uniform(1.48, 1.52), random.uniform(-1.84, -1.80), random.uniform(-1.59, -1.55), 0.0]
			
# 			move_group.set_joint_value_target(joint_goal)
			
# 			move_group.go(wait=True)
			
# 			move_group.stop()
			
# 			print('moved to'+'pose'+str(i))
# 			print(type(joint_goal))
# 			print(joint_goal)
   
# 			# joint_goal_arr = np.empty(6)
# 			# joint_goal_arr = joint_goal[:,0]
# 			# print(type(joint_goal_arr))
# 			# print(joint_goal_arr)
# 			####
# 			# joint_goal_arr2 = np.array(joint_goal)
# 			# # print(type(joint_goal_arr2))
# 			# # print(joint_goal_arr2)
# 			# joint_goal_arr2_c = joint_goal_arr2.reshape([-1,1])
# 			# print(type(joint_goal_arr2_c))
# 			# print(joint_goal_arr2_c.shape)
# 			# print(joint_goal_arr2_c)
   
# 			# joint_goal_str = str(joint_goal_arr2_c)[1:-1]
# 			# joint_goal_str = str(joint_goal)[1:-1]******
# 			# print(type(joint_goal_str))
# 			#[]ありで出力される
# 			#####*****************で囲まれる部分だけでいい
# 			theta.append(joint_goal)
# 			print(type(theta))
# 			print(theta)
# 			####******************
# 			#[]なしで出力される
# 			# theta.append(joint_goal_str)*****

# 		# theta_str = str(theta)[1:-1]****
# 		######*************
# 		writer = csv.writer(f)
# 		writer.writerows(theta)
# 		######*************
# 		# print(theta_str)
# 		# print(type(theta_str))
# 		# f.write(theta_str)***
# 		# f.write(str(theta_str))
# 		sleep(1)
# 	# f.close()

def MovetoRandomHist():
    theta = []
    pose_data = []
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    with open ('./testrandomtheta.csv', 'w') as f1, open ('./testrandompose.csv', 'w') as f2:
        for i in range (10):
            joint_goal = [random.uniform(1.48, 1.65), random.uniform(-1.33, -1.10), random.uniform(1.37, 1.67), random.uniform(-1.83, -1.75), -1.57, 0.0]
            move_group.set_joint_value_target(joint_goal)
            move_group.go(wait=True)
            move_group.stop()
            theta.append(joint_goal)
            # pose = move_group.get_current_pose().pose
            posex = move_group.get_current_pose().pose.position.x
            posey = move_group.get_current_pose().pose.position.y
            posez = move_group.get_current_pose().pose.position.z
            pose_data.append([posex, posey, posez])
            # pose_data = pd.DataFrame(pose)
        writer1 = csv.writer(f1)
        writer1.writerows(theta)
        writer2 = csv.writer(f2)
        writer2.writerows(pose_data)

def matome():
	# MovetoGoal()
	# MovetoRandomGoal()
	# Move()
	MovetoRandomHist()


if __name__ == "__main__":
	rospy.init_node('joint_planner2')
	matome()