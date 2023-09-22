#! /usr/bin/env python 
# coding: UTF-8 

import sys
from math import pi

import moveit_commander
import rospy
import numpy as np

def up():
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_max_velocity_scaling_factor(value=1.0)
    move_group.set_max_acceleration_scaling_factor(value=1.0)
    joint_goal_deg = [0, -90, 0, -90, 0, 0]
    joint_goal = [x * pi/180 for x in joint_goal_deg]
    move_group.set_joint_value_target(joint_goal)
    print(joint_goal_deg)
    print(joint_goal)
    move_group.go(wait=True)
    print('moved to up pose')
    moveit_commander.roscpp_shutdown()
    
if __name__ == "__main__":
    rospy.init_node('up_pose')
    up()