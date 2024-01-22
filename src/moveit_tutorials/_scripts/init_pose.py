#!/usr/bin/env python
# coding: UTF-8

import sys
from math import pi

import moveit_commander
import rospy

# import random
import numpy as np

from time import sleep

def Move():
	moveit_commander.roscpp_initialize(sys.argv)
	move_group = moveit_commander.MoveGroupCommander("manipulator")
	move_group.set_max_velocity_scaling_factor(value=0.02)
	move_group.set_max_acceleration_scaling_factor(value=0.02)
 
	###-----------------------kokokara2023nov
	# joint_goal_deg = [103.32, -93.24, 108.66, -14.49, 86.91, -181.61] ### initA2-1
	# joint_goal_deg = [100.63, -80.70, 95.28, -13.65, 84.23, -180.54] ### initA2-2
	# joint_goal_deg = [79.57, -90.20, 105.72, -14.37, 63.18, -181.03] ### initA2-3
	# joint_goal_deg = [74.99, -106.83, 119.54, -11.41, 58.64, -181.30] ### initA2-4
 
	# joint_goal_deg = [93.90, -78.89, 94.14, -15.18, 93.40, -0.12] ###initD1
	# joint_goal_deg = [89.77, -80.37, 95.96, -15.50, 89.26, -0.13] ###initD2
	# joint_goal_deg = [85.26, -79.90, 95.38, -15.41, 84.76, -0.13] ###initD4
	joint_goal_deg = [83.31, -83.32, 99.42, -16.03, 82.81, -0.17] ###initD5
	# joint_goal_deg = [84.44, -72.40, 85.74, -13.28, 83.97, -0.07] ###initD7
	# joint_goal_deg = [93.34, -68.62, 80.47, -11.80, 92.88, -0.02] ###initD10
 
	# joint_goal_deg = [98.11, -91.94, 108.49, -16.51, 97.60, -0.23] ###initE1 
	# joint_goal_deg = [92.72, -94.83, 111.20, -16.29, 92.21, -0.27] ###initE2 akan
	# joint_goal_deg = [97.46, -88.03, 104.57, -16.46, 96.94, -0.20] ###initE3 akan
	# joint_goal_deg = []
	# joint_goal_deg = [83.26, -99.08, 114.85, -15.66, 82.76, -0.32] ###initE5
	# joint_goal_deg = [84.27, -91.85, 108.41, -16.44, 83.76, -0.26] ###initE7
	# joint_goal_deg = [84.78, -87.43, 103.93, -16.41, 84.26, -0.21] ###initE8
	# joint_goal_deg = [92.24, -84.27, 100.48, -16.13, 91.73, -0.18] ###initE10
	# joint_goal_deg = [96.63, -82.02, 97.90, -15.82, 96.11, -0.14] ###initE11
	# joint_goal_deg = [97.15, -85.89, 102.27, -16.33, 96.63, -0.16] ###initE12 akan
	
	joint_goal = [x * pi/180 for x in joint_goal_deg]
	print(joint_goal_deg)
	move_group.set_joint_value_target(joint_goal)
	print(joint_goal)
	move_group.go(wait=True)
	print('moved to initial pose')
	moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
	rospy.init_node('init_pose')
	Move()