#!/usr/bin/env python
# coding: UTF-8

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import sys
from math import pi
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

def joint_control():
    joint_goal_deg = [89.06, -75.86, 90.32, -14.37, 88.57, -0.09] ### desired 1 v2
    joint_goal = [x * pi/180 for x in joint_goal_deg]
    move_group.set_joint_value_target(joint_goal)
    move_group.go(wait=True)
    
def pose_control():
    pose = Pose()
    ### desired1 v3に相当
    pose.position.x = -0.10894
    pose.position.y = 0.349287
    pose.position.z = 0.245499

    pose.orientation.x = -0.00370324
    pose.orientation.y = 0.712185
    pose.orientation.z = 0.701975
    pose.orientation.w = -0.00306745
    
    euler_angles = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    print('euler angles -----------------')
    print(euler_angles)
    
    move_group.set_pose_target(pose)
    move_group.go(wait=True)
    print('moved')
    
    roll = pi/2
    pitch = 0
    yaw = pi+0.1
    
    # ### オイラー角度指定 (set_rpy_target()使用)
    # move_group.set_rpy_target([roll, pitch, yaw])
    
    ## オイラー角指定 (TF変換使用)
    quaterinion = quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = quaterinion[0]
    pose.orientation.y = quaterinion[1]
    pose.orientation.z = quaterinion[2]
    pose.orientation.w = quaterinion[3]
    move_group.set_pose_target(pose)
    print(quaterinion)
    print(quaterinion[0])
    print(quaterinion[1])
    print(quaterinion[2])
    print(quaterinion[3])
    move_group.go(wait=True)
    print('moved (using euler)')

def info():
    planning_frame = move_group.get_planning_frame()
    print('----planning frame----', planning_frame)
    joint_values = move_group.get_current_joint_values()
    print('----joint values----', joint_values)
    current_pose = move_group.get_current_pose().pose
    print('----pose values----', current_pose)
    current_rpy = move_group.get_current_rpy()
    print('----rpy values----', current_rpy)
    # orientation_tolerance = move_group.get_goal_orientation_tolerance()
    # print('----orientation tolerance----', orientation_tolerance)
    # position_tolerance = move_group.get_goal_position_tolerance()
    # print('----position tolerance----', position_tolerance)
    

if __name__ == "__main__":
    rospy.init_node('motion_test')
    moveit_commander.roscpp_initialize(sys.argv)
    moveit_commander.RobotCommander()
    moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    # move_group.set_max_velocity_scaling_factor(value = 0.8) ### 動作の速度と加速度を決定
    # move_group.set_max_acceleration_scaling_factor(value = 0.8)
    # move_group.set_goal_orientation_tolerance(value = 0.0001) ### 位置と姿勢の重み変更
    # move_group.set_goal_position_tolerance(value = 0.001)
    
    # joint_control()
    
    pose_control()
    
    info()