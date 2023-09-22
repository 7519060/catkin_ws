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
	
	# joint_goal2_deg = [92.42, -92.39, 107.23, -13.87, 76.01, -180.80] ###上のdesiredをデータセットに合わせてwrist3変えたやつ
	# joint_goal2_deg = [92.43, -92.23, 107.70, -14.47, 76.03, -180.79] ###dsr_poseから出たやつ
	# joint_goal2_deg = [91.97, -106.48, 128.12, -20.61, 75.58, -180.92]
	# joint_goal2_deg = [119.69, -87.58, 111.64, -23.17, 103.25, -180.29]
	# joint_goal2 = [x * pi/180 for x in joint_goal2_deg]
 
	joint_goal2 = [1.60414, -1.90515, 2.10868, -0.18457, 1.31903, -3.15687] ###desired2
	# joint_goal2 = [2.08805, -1.57296, 1.84294, -0.25174, 1.80266, -3.14726] ###desired3
	# joint_goal2 = [1.613207, -1.609717, 1.879719, -0.252549, 1.326974, -3.155381] ###desired1
 
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