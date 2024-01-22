#!/usr/bin/env python
# coding: UTF-8

import rospy
import actionlib
from std_msgs.msg import Int32
from moveit_tutorials.msg import FlagAction, FlagResult
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import moveit_commander
from math import pi

rospy.init_node('flag_server')
moveit_commander.roscpp_initialize([])

move_group = moveit_commander.MoveGroupCommander("manipulator")

server = actionlib.SimpleActionServer('flag', FlagAction, lambda goal: execute(goal), False)
server.start()
print('ready to serve')

def execute(goal):
    current_pose = move_group.get_current_pose().pose

    if goal.flag == 0:
        current_pose.orientation = get_quaternion(pi/2, 0, 0.1)
        print('orientation +0.1')
    elif goal.flag == 1:
        current_pose.orientation = get_quaternion(pi/2, 0, -0.1)
        print('orientation -0.1')

    move_group.set_pose_target(current_pose)
    move_group.go(wait=True)

    result = FlagResult()
    server.set_succeeded(result)

def get_quaternion(roll, pitch, yaw):
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return quaternion

rospy.spin()
