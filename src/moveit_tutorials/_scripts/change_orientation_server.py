#!/usr/bin/env python
# coding: UTF-8

import rospy
import actionlib
from std_msgs.msg import Int32
from moveit_tutorials.msg import FlagAction, FlagResult
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import moveit_commander
import sys
from math import pi

def change_orientation(goal):
    pose = Pose()
    pose = move_group.get_current_pose().pose
    # current_joint_values = move_group.get_current_joint_values()
    if goal.flag == 0:
        # current_joint_values[4] += 0.1
        # print('wrist 2 joint +0.1')
        roll = pi/2
        pitch = 0
        yaw = pi+0.1
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        print('yaw = +0.1')
    elif goal.flag == 1:
        # current_joint_values[4] += -0.1
        # print('wrist 2 joint -0.1')
        roll = pi/2
        pitch = 0
        yaw = pi-0.1
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        print('yaw = -0.1')
    # move_group.set_joint_value_target(current_joint_values)
    move_group.set_pose_target(pose)
    move_group.go(wait=True)
    result = FlagResult()
    change_ori_server.set_succeeded(result)
    
def planning_scene():
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = -0.03
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(100, 100, 0.001))
    print('added box')
    
    box_pose2 = PoseStamped()
    box_pose2.header.frame_id = "base_link"
    box_pose2.pose.orientation.w = 1.0
    box_pose2.pose.position.x = 0
    box_pose2.pose.position.y = -0.12
    box_pose2.pose.position.z = 0
    box_name2 = "box2"
    scene.add_box(box_name2, box_pose2, size=(100, 0.01, 100))
    print('added box2')
    
    box_pose3 = PoseStamped()
    box_pose3.header.frame_id = "base_link"
    box_pose3.pose.orientation.w = 1.0
    box_pose3.pose.position.x = 0
    box_pose3.pose.position.y = 0.4
    box_pose3.pose.position.z = 0
    box_name3 = "box3"
    scene.add_box(box_name3, box_pose3, size=(1, 0.3, 0.2))
    print('added box3')
    
    box_pose4 = PoseStamped()
    box_pose4.header.frame_id = "base_link"
    box_pose4.pose.orientation.w = 1.0
    box_pose4.pose.position.x = 0.15
    box_pose4.pose.position.y = 0
    box_pose4.pose.position.z = 0
    box_name4 = "box4"
    scene.add_box(box_name4, box_pose4, size=(0.01, 100, 100))
    print('added box4')
    
# def e2q():
#     roll = pi/2
#     pitch = 0.1
#     yaw = 0
#     q = quaternion_from_euler(roll, pitch, yaw)
#     print(q)
#     print(q[0])
#     print(q[1])
#     print(q[2])
#     print(q[3])
#     print('ここまでpitch +0.1')
#     roll = pi/2
#     pitch = 0
#     yaw = 0
#     q = quaternion_from_euler(roll, pitch, yaw)
#     print(q)
#     print(q[0])
#     print(q[1])
#     print(q[2])
#     print(q[3])
#     print('ここまでpitch 0')
#     roll = pi/2
#     pitch = -0.1
#     yaw = 0
#     q = quaternion_from_euler(roll, pitch, yaw)
#     print(q)
#     print(q[0])
#     print(q[1])
#     print(q[2])
#     print(q[3])
#     print('ここまでpitch -0.1')

if __name__ == '__main__':
    rospy.init_node('change_ori_server')
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    planning_scene()
    move_group = moveit_commander.MoveGroupCommander('manipulator')
    rospy.loginfo('change orientation server started')
    change_ori_server = actionlib.SimpleActionServer('flag', FlagAction, change_orientation, False)
    change_ori_server.start()
    rospy.spin()
    # e2q()