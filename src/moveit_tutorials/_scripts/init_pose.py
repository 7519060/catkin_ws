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
	# joint_goal = [1.514, -1.165, 0.746, 0.433, 0.986, -3.16]#screwdriver ui camera  drum initial pose 2
	# joint_goal = [1.511, -1.2, 0.805, 0.411, 0.982, -3.123]#screwdriver ui camera drum initial pose 3
	# joint_goal_deg = [101.32, -93.41, 108.22, -13.98, 84.40, -179.07] ###initial pose 1.2
	# joint_goal_deg = [91.78, -97.99, 112.28, -13.40, 74.87, -178.89] ###initial pose 2.2
	# joint_goal_deg = [92.22, -92.46, 107.34, -13.99, 75.31, -178.94] ###initial pose 3.2
	# joint_goal_deg = [98.58, -88.36, 103.23, -14.03, 81.68, -179.08] ###initial pose 4.2
	# joint_goal_deg = [98.29, -89.41, 104.31, -14.08, 81.38, -179.06] ###initial pose 4.3
	
	# joint_goal_deg = [97.70, -91.96, 106.84, -14.04, 80.78, -180.96] ###initial pose 5.2
	# joint_goal_deg = [97.91, -91.86, 106.70, -13.90, 81.51, -180.70] ###initial pose 5.3 fixed orientation of init5.2
	# joint_goal_deg = [97.91, -91.70, 107.19, -14.53, 81.51, -180.68] ###initial pose 5.4
	# joint_goal_deg = [97.90, -91.76, 107.02, -14.30, 81.50, -180.65] ###initial pose 5.5
	# joint_goal_deg = [88.41, -99.10, 113.20, -13.16, 71.51, -181.17] ###initial pose 6.2 same as wrist3=178.81
	# joint_goal_deg = [88.64, -99.02, 113.09, -13.04, 72.25, -180.93] ###initial pose 6.3
	# joint_goal_deg = [88.65, -98.86, 113.61, -13.70, 72.26, -180.92] ###initial pose 6.4
	# joint_goal_deg = [86.37, -92.71, 107.55, -13.91, 69.45, -180.38] ###initial pose 7.2 same as wrist3 = 178.82
	# joint_goal_deg = [86.57, -92.65, 107.45, -13.77, 70.16, -180.91] ###initial pose 7.3 fixed orientation of init7.2
	# joint_goal_deg = [86.57, -92.50, 107.95, -14.41, 70.17, -180.90] ###initial pose 7.4
	# joint_goal_deg = [94.99, -89.88, 104.79, -14.04, 78.07, -180.99] ###initial pose 8.2 same as wrist3 = 178.99
	# joint_goal_deg = [95.19, -89.80, 104.68, -13.92, 78.77, -180.73] ###initial pose 8.3 fixed orientation of init 8.2
	# joint_goal_deg = [95.19, -89.64, 105.15, -14.54, 78.78, -180.72] ###initial pose 8.4
 
	# joint_goal_deg = [97.59, -109.62, 121.10, -10.45, 81.24, -180.76] ### init B1
	# joint_goal_deg = [101.50, -105.87, 118.68, -11.81, 85.15, -180.66] ### initB2
	# joint_goal_deg = [93.03, -114.17, 123.71, -8.45, 76.72, -180.88] ### initB3
	# joint_goal_deg = [87.87, -113.07, 123.12, -8.89, 71.55, -180.98] ### initB4
	# joint_goal_deg = [81.85, -118.59, 125.89, -6.02, 65.57, -181.16] ### initB5
	# joint_goal_deg = [87.29, -109.03, 120.75, -10.57, 70.95, -180.96] ### initB6
	# joint_goal_deg = [83.10, -109.10, 120.79, -10.48, 66.76, -181.05] ### initB7
	# joint_goal_deg = [93.24, -105.46, 118.40, -11.87, 76.88, -180.81] ### initB8
	joint_goal_deg = [96.11, -102.09, 115.95, -12.84, 79.75, -180.72] ### initB9
	# joint_goal_deg = [99.02, -102.20, 116.03, -12.83, 82.66, -180.66] ### initB10
 
	# joint_goal_deg = [85.17, -100.40, 114.85, -13.36, 68.79, -181.01] ### initA1
	# joint_goal_deg = [88.79, -95.14, 110.39, -14.23, 72.40, -180.88] ### initA2
	# joint_goal_deg = [90.34, -88.52, 104.00, -14.48, 73.95, -180.79] ### initA3
	# joint_goal_deg = [93.16, -89.52, 105.02, -14.52, 76.75, -180.77] ### initA4
	# joint_goal_deg = [96.90, -85.22, 100.47, -14.30, 80.50, -180.67] ### initA5
	# joint_goal_deg = [95.09, -94.46, 109.78, -14.34, 78.69, -180.78] ### initA6
	# joint_goal_deg = [100.10, -92.84, 108.80, -15.24, 84.38, -180.52] ### initA7
	# joint_goal_deg = [93.67, -92.82, 108.78, -15.20, 77.95, -180.60] ### initA8
	# joint_goal_deg = [93.93, -92.58, 108.87, -15.29, 78.83, -180.72] ### initA11
	# joint_goal_deg = [96.75, -88.45, 106.06, -16.67, 82.24, -180.61] ### initA13
	# joint_goal_deg = [86.00, -93.48, 105.30, -10.76, 69.61, -180.93] ###initA14
 
	# joint_goal_deg = [127.19, -88.09, 103.50, -14.35, 110.82, -180.16] ### init C1
	# joint_goal_deg = [123.7, -89.49, 104.96, -14.41, 107.34, -180.24] ### init C2
	# joint_goal_deg = [121.08, -82.60, 97.46, -13.81, 104.73, -180.23] ### initC9
	# joint_goal_deg = [] ### initC
	# joint_goal_deg = [] ### initC
 
	joint_goal = [x * pi/180 for x in joint_goal_deg]
	print(joint_goal_deg)
	# joint_goal = [1.69793, -1.48943, 1.77316, -0.26622, 1.41098, -3.15329] ###random pose 37
	# joint_goal = [1.70112, -1.67157, 1.89458, -0.20832, 1.41337, -3.15439] ###random pose 207
	# joint_goal = [1.52361, -1.64641, 1.95306, -0.28967, 1.23774, -3.15705] ###random pose 107
	move_group.set_joint_value_target(joint_goal)
	print(joint_goal)
	move_group.go(wait=True)
	print('moved to initial pose')
	moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
	rospy.init_node('init_pose')
	Move()