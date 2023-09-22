#!/usr/bin/env python
#coding: UTF-8

import rospy
import numpy as np
import cv2
import moveit_commander
import sys

rospy.init_node('movegroup_tutorial')
moveit_commander.roscpp_initialize(sys.argv)
move_group = moveit_commander.MoveGroupCommander('manipulator')
move_group.set_max_velocity_scaling_factor(value=0.1)
move_group.set_max_acceleration_scaling_factor(value=0.1)
print(move_group.get_current_joint_values())
current_joint_values = np.array(move_group.get_current_joint_values())
print(current_joint_values)
pose = move_group.get_current_pose().pose
print(pose)
# pose_data = []
# for i in range(10):
#     pose = move_group.get_current_pose()