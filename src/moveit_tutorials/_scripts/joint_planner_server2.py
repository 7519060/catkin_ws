#!/usr/bin/env python
# coding: UTF-8

import sys
from math import pi

import moveit_commander
import rospy

import random
# import numpy as np

import actionlib
from moveit_tutorials.msg import EmptyAction, EmptyGoal, EmptyResult
import csv

# from time import sleep

def MovetoRandomGoal(goal):
	theta = []
	pose_data = []
	moveit_commander.roscpp_initialize(sys.argv)
	move_group = moveit_commander.MoveGroupCommander('manipulator')
 
	# base joint 動かす範囲、目標角度、刻み幅
	base = random.uniform(1.545, 1.748)
	while abs(base - 1.678) < 0.01:
		base = random.uniform(1.545, 1.748)

	# shoulder joint 動かす範囲、目標角度、刻み幅
	shoulder = random.uniform(-1.705, -1.511)
	while abs(shoulder - (-1.607)) < 0.01:
		shoulder = random.uniform(-1.705, -1.511)

	# elbow joint 動かす範囲、目標角度、刻み幅
	elbow = random.uniform(1.769, 1.955)
	while abs(elbow - 1.867) < 0.01:
		elbow = random.uniform(1.769, 1.955)

	# wrist1 joint 動かす範囲、目標角度、刻み幅
	wrist1 = random.uniform(-0.295, -0.197)
	while abs(wrist1 - (-0.245)) < 0.005:
		wrist1 = random.uniform(-0.295, -0.197)

	# wrist2 joint 動かす範囲、目標角度、刻み幅
	wrist2 = random.uniform(1.251, 1.503)
	while abs(wrist2 - 1.383) < 0.01:
		wrist2 = random.uniform(1.251, 1.503)

	# wrist3 joint 動かす範囲、目標角度、刻み幅
	wrist3 = random.uniform(3.157, 3.162)
	while abs(wrist3 - 3.159) < 0.00025:
		wrist3 = random.uniform(3.157, 3.162)

	# リストの生成
	joint_goal = [base, shoulder, elbow, wrist1, wrist2, wrist3]
 
	# joint_goal = [random.uniform(1.50, 1.63), random.uniform(-1.31, -1.12), random.uniform(1.39, 1.65), random.uniform(-1.85, -1.77), -1.57, 0.0]
	#上のデータセットから更に広げる下限-0.02rad上限+0.02rad
	# joint_goal = [random.uniform(1.48, 1.65), random.uniform(-1.33, -1.10), random.uniform(1.37, 1.67), random.uniform(-1.87, -1.75), -1.57, 0.0]
	# screwdriver
	# joint_goal = [random.uniform(1.646, 1.823), random.uniform(-1.317, -1.145), random.uniform(1.558, 1.802), random.uniform(-3.593, -3.504), random.uniform(-1.806, -1.639), 0.0]
	# # screwdriver + ids camera
	# joint_goal = [random.uniform(1.485, 1.671), random.uniform(-1.151, -1.395), random.uniform(0.732, 1.064), random.uniform(0.323, 0.437), random.uniform(0.956, 1.142), random.uniform(-3.125, -3.121)]
	
	move_group.set_joint_value_target(joint_goal)
	move_group.go(wait=True)
	move_group.stop()
	print('moved to random pose')
	theta.append(joint_goal)
	posex = move_group.get_current_pose().pose.position.x
	posey = move_group.get_current_pose().pose.position.y
	posez = move_group.get_current_pose().pose.position.z
	pose_data.append([posex, posey, posez])
	filename2 = './rndmth_result/random_theta_result.csv'
	filename3 = './rndmth_result/random_pose.csv'
	with open (filename2, 'a') as f2, open (filename3, 'a') as f3:
		writer2 = csv.writer(f2)
		writer2.writerows(theta)
		writer3 = csv.writer(f3)
		writer3.writerows(pose_data)
	result2 = EmptyResult()
	motion_server2.set_succeeded(result2)



if __name__ == "__main__":
	rospy.init_node('random_motion_server', anonymous=True)
	rospy.loginfo('random motion server started. ready to serve.')
	motion_server2 = actionlib.SimpleActionServer('ugoke_random', EmptyAction, MovetoRandomGoal, False)
	motion_server2.start()
	rospy.spin()
