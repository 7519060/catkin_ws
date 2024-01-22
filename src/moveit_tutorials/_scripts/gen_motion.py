#!/usr/bin/env python
# coding: UTF-8

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import sys
from math import pi
# import tf
from time import sleep

rospy.init_node('generate_motion_node')
moveit_commander.roscpp_initialize(sys.argv)
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_max_velocity_scaling_factor(value = 0.05)
move_group.set_max_acceleration_scaling_factor(value = 0.05)
move_group.set_goal_orientation_tolerance(value = 0.00001)
move_group.set_goal_position_tolerance(value = 0.001)
scene = moveit_commander.PlanningSceneInterface(synchronous=True)
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
box_pose2.pose.position.y = -0.09
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

joint_goal_deg = [89.06, -75.86, 90.32, -14.37, 88.57, -0.09]
joint_goal = [x * pi/180 for x in joint_goal_deg]
move_group.set_joint_value_target(joint_goal)
move_group.go(wait=True)

current_pose = move_group.get_current_pose().pose
print('current')
print(current_pose)

###ここから位置姿勢指定で制御
pose = Pose()

# pose.position.x = -0.10652
# pose.position.y = 0.35120
# pose.position.z = 0.25000

###desired1 v3 for gazebo
pose.position.x = -0.108853
pose.position.y = 0.348864
pose.position.z = 0.246567

# q = tf.transformations.quaternion_from_euler(3*pi/2, -pi, 0)
# pose.orientation.x = q[0]
# pose.orientation.y = q[1]
# pose.orientation.z = q[2]
# pose.orientation.w = q[3]

# pose.orientation.x = -0.0000528
# pose.orientation.y = 0.707227
# pose.orientation.z = 0.706987
# pose.orientation.w = 0.0000959

### desired 1 v3 +-25mm for gazebo
pose.orientation.x = -0.0000529984
pose.orientation.y = 0.6997480695
pose.orientation.z = 0.7143896878
pose.orientation.w = 0.0001016077

move_group.set_pose_target(pose)

plan = move_group.plan()
print('planning done')
sleep(5)
print('sleep owaowari')
move_group.execute(plan, wait=True)

# move_group.go(wait = True)

renew_pose = move_group.get_current_pose().pose
print('renew pose')
print(renew_pose)

renew_joint = move_group.get_current_joint_values()
print('renew joint')
print(renew_joint)
renew_joint_deg = [y*180/pi for y in renew_joint]
print('renew joint deg')
print(renew_joint_deg)

ori_tole = move_group.get_goal_orientation_tolerance()
print(ori_tole)
pos_tole = move_group.get_goal_position_tolerance()
print(pos_tole)