#!/usr/bin/env python
# coding: UTF-8

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose

import sys
from math import pi

import random 

import actionlib
from moveit_tutorials.msg import EmptyAction, EmptyResult
import csv

from time import sleep


# def random_pose(goal):
#     joint_data = []
#     pose_data = []
#     moveit_commander.roscpp_initialize(sys.argv)
#     moveit_commander.RobotCommander()
#     moveit_commander.PlanningSceneInterface()
#     group_name = "manipulator"
#     move_group = moveit_commander.MoveGroupCommander(group_name)
#     move_group.set_max_velocity_scaling_factor(value = 0.1)
#     move_group.set_max_acceleration_scaling_factor(value = 0.1)
    
#     ###一旦desired pose に移動
#     joint_goal_deg = [92.22, -92.46, 107.34, -13.99, 75.31, -181.04] 
#     joint_goal = [x * pi/180 for x in joint_goal_deg]
#     move_group.set_joint_value_target(joint_goal)
#     move_group.go(wait=True)
    
#     print('ittan desired pose')
    
#     pose = Pose()
    
#     pose.position.x = -0.14351
#     pose.position.y = 0.26766
#     pose.position.z = 0.2527
    
#     ### 0613以降使用するorientation 実機でdesired poseに持っていったときのmoveitで表示されるorientationを参考にする
#     ### gazeboでdesired poseにもっていったときのposition と orientationはずれてるので使用しない
#     pose.orientation.x = -0.703804
#     pose.orientation.y = -0.105292
#     pose.orientation.z = 0.0967695
#     pose.orientation.w = 0.695852 
    
#     # ランダムなポーズを目標として設定
#     move_group.set_pose_target(pose)

#     # 移動計画と実行
#     #wait=Trueにすることで動作が終わるまで待機
#     move_group.go(wait=True)
    
#     print('moved to random pose')
    
#     joint_values = move_group.get_current_joint_values()
    
#     print(joint_values) ###この形[]でした
#     print('desired')
#     print(pose)
    
#     current_pose = move_group.get_current_pose().pose
#     print('current')
#     print(current_pose)
    
#     joint_data.append(joint_values)
    
#     pose_data.append([current_pose.position.x, current_pose.position.y, current_pose.position.z])
    
#     ### gazebo+rviz(moveit)のとき手先posiotionもorientationもずれてるので注意
#     ### 実際には上のget_current_poseとget_current_jointで取得されてる値怪しい->jointの方はrobot_info.pyから取得してみたら大丈夫そうだった
#     ### get_current_poseで取得される手先座標は怪しそう（距離データ取得くらいにしか使わない）
#     ### vel_servo_sig.py内で距離データ取得するときjoint_planner_serverで取得したdesired_pose.csv読んでるけどこれ多分正しくない（gazeboから取得した値だから）
#     ### vel_servo_sig.pyで距離デーら取得するとき目標位置直打ちしたほうがいいかも
#     filename = './dsrth_result/desired_theta_result.csv'
#     filename2 = './dsrth_result/desired_pose.csv'
    
#     with open (filename, 'a') as f, open(filename2, 'a') as f2:
#         writer = csv.writer(f)
#         writer.writerows(joint_data)
#         writer2 = csv.writer(f2)
#         writer2.writerows(pose_data)
    
#     result2 = EmptyResult()
#     motion_server2.set_succeeded(result2)

# if __name__ == "__main__":
#     rospy.init_node('random_motion_server')
#     rospy.loginfo('Random motion sever started. Ready to serve.')
#     motion_server2 = actionlib.SimpleActionServer('ugoke_random', EmptyAction, random_pose, False)
#     motion_server2.start()
#     rospy.spin()
    
rospy.init_node('random_motion_server')
joint_data = []
pose_data = []
moveit_commander.roscpp_initialize(sys.argv)
moveit_commander.RobotCommander()
moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
move_group = moveit_commander.MoveGroupCommander(group_name)
move_group.set_max_velocity_scaling_factor(value = 0.1)
move_group.set_max_acceleration_scaling_factor(value = 0.1)

###一旦desired pose に移動
# joint_goal_deg = [92.22, -92.46, 107.34, -13.99, 75.31, -181.04] 

# ### desired 1
# joint_goal_deg = [92.43, -92.23, 107.7, -14.47, 76.03, -180.79]
# ### desired 2
# joint_goal_deg = [91.97, -106.48, 128.12, -20.61, 75.58, -180.92]
### desired 3
joint_goal_deg = [119.69, -87.58, 111.64, -23.17, 103.25, -180.29]

joint_goal = [x * pi/180 for x in joint_goal_deg]
print(joint_goal)
move_group.set_joint_value_target(joint_goal)
move_group.go(wait=True)

print('ittan desired pose')

pose = Pose()

# ### desired 1
# pose.position.x = -0.14351
# pose.position.y = 0.26766
# pose.position.z = 0.2527

# ### desired 2
# pose.position.x = -0.13969
# pose.position.y = 0.20210
# pose.position.z = 0.2527

### desired 3
pose.position.x = -0.22148
pose.position.y = 0.19982
pose.position.z = 0.2527

# ### 0613以降使用するorientation 実機でdesired poseに持っていったときのmoveitで表示されるorientationを参考にする
# ### gazeboでdesired poseにもっていったときのposition と orientationはずれてるので使用しない
# pose.orientation.x = -0.703804
# pose.orientation.y = -0.105292
# pose.orientation.z = 0.0967695
# pose.orientation.w = 0.695852 

# ### desired 2
# pose.orientation.x = -0.705671
# pose.orientation.y = -0.105665
# pose.orientation.z = 0.0958822
# pose.orientation.w = 0.694025 

### desired 3
pose.orientation.x = -0.70518
pose.orientation.y = -0.104907
pose.orientation.z = 0.0969791
pose.orientation.w = 0.694486

# ランダムなポーズを目標として設定
move_group.set_pose_target(pose)

# 移動計画と実行
#wait=Trueにすることで動作が終わるまで待機
move_group.go(wait=True)

print('moved to random pose')

joint_values = move_group.get_current_joint_values()

print(joint_values) ###この形[]でした
print('desired')
print(pose)

current_pose = move_group.get_current_pose().pose
print('current')
print(current_pose)

joint_data.append(joint_values)

pose_data.append([current_pose.position.x, current_pose.position.y, current_pose.position.z])

### gazebo+rviz(moveit)のとき手先posiotionもorientationもずれてるので注意
### 実際には上のget_current_poseとget_current_jointで取得されてる値怪しい->jointの方はrobot_info.pyから取得してみたら大丈夫そうだった
### get_current_poseで取得される手先座標は怪しそう（距離データ取得くらいにしか使わない）
### vel_servo_sig.py内で距離データ取得するときjoint_planner_serverで取得したdesired_pose.csv読んでるけどこれ多分正しくない（gazeboから取得した値だから）
### vel_servo_sig.pyで距離デーら取得するとき目標位置直打ちしたほうがいいかも
filename = './dsrth_result/desired_theta_result.csv'
filename2 = './dsrth_result/desired_pose.csv'

with open (filename, 'a') as f, open(filename2, 'a') as f2:
    writer = csv.writer(f)
    writer.writerows(joint_data)
    writer2 = csv.writer(f2)
    writer2.writerows(pose_data)
