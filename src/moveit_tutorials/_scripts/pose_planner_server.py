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


def random_pose(goal):
    joint_data = []
    pose_data = []
    moveit_commander.roscpp_initialize(sys.argv)
    moveit_commander.RobotCommander()
    moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    ###一旦desired pose に移動
    # joint_goal_deg = [92.22, -92.46, 107.34, -13.99, 75.31, -181.04] 
    
    # ### desired 1
    # joint_goal_deg = [92.43, -92.23, 107.7, -14.47, 76.03, -180.79]
    # ### desired 2
    # joint_goal_deg = [91.97, -106.48, 128.12, -20.61, 75.58, -180.92]
    ### desired 3
    joint_goal_deg = [119.69, -87.58, 111.64, -23.17, 103.25, -180.29]
    
    joint_goal = [x * pi/180 for x in joint_goal_deg]
    move_group.set_joint_value_target(joint_goal)
    move_group.go(wait=True)
    
    print('ittan desired pose')
    
    pose = Pose()
    
    # # *********************
    # fixed_orientation = move_group.get_current_pose().pose.orientation
    # sleep(0.1)
    # print(fixed_orientation)
    ###なんかだんだんずれていってた
    # # **********************
    
    # pose.position.x = random.uniform(-0.175, -0.135) ###default -0.18, -0.13
    # pose.position.y = random.uniform(0.243, 0.283) ###default 0.238, 0.288
    # pose.position.z = random.uniform(0.248, 0.262) ###default value 0.245 0.265
    
    # pose.position.x = random.uniform(-0.18, -0.13) ###default -0.18, -0.13
    # pose.position.y = random.uniform(0.238, 0.288) ###default 0.238, 0.288
    # pose.position.z = random.uniform(0.245, 0.265) ###default value 0.245 0.265
    
    # # init 3.2 の座標入力してこれを目標としたデータセット作成
    # pose.position.x = random.uniform(-0.167, -0.117) ###default -0.18, -0.13
    # pose.position.y = random.uniform(0.245, 0.295) ###default 0.238, 0.288
    # pose.position.z = random.uniform(0.245, 0.265) ###default value 0.245 0.265
    
    ### gazeboでvalid_joint_values取得のためにsimするとき、poseをrandomで決めて入力するが、実機の手先座標と異なる
    ### 実機 desired pose[-0.142, 0.270, 0.255]だがgazebo desired pose [-0.14351, 0.26766, 0.2527]
    ### その差は[-0.00151, -0.00234, -0.0023]
    ### gazebo desired pose [-0.14351, 0.26766, 0.2527]の値を元に各成分+-25mm, +-25mm, +-10mmでrandom.uniformする
    
    # ### desired 1
    # pose.position.x = random.uniform(-0.16851, -0.11851) 
    # pose.position.y = random.uniform(0.24266, 0.29266) 
    # pose.position.z = random.uniform(0.2427, 0.2627) 
    
    # ### desired 2
    # pose.position.x = random.uniform(-0.1647, -0.1147) 
    # pose.position.y = random.uniform(0.1771, 0.2271) 
    # pose.position.z = random.uniform(0.2427, 0.2627) 
    
    ### desired 3
    pose.position.x = random.uniform(-0.24648, -0.19648) 
    pose.position.y = random.uniform(0.19982, 0.22482) 
    pose.position.z = random.uniform(0.2427, 0.2627) 
    
    # # ***********************
    # pose.orientation = fixed_orientation
    # # ***********************
    # pose.orientation.w = 1.0  # ランダムな姿勢を設定する場合はこれも変更する必要
    # pose.orientation = pose.orientation
    
    ### 0613まで使用していたorientation
    # pose.orientation.x = -0.705175755728  ###回転軸x
    # pose.orientation.y = -0.1097672512
    # pose.orientation.z = 0.0981031719625
    # pose.orientation.w = 0.693580616618 ###回転角度を表す値で回転角度のコサインの半分
    
    ### 0613以降使用するorientation 実機でdesired poseに持っていったときのmoveitで表示されるorientationを参考にする
    ### gazeboでdesired poseにもっていったときのposition と orientationはずれてるので使用しない
    
    # ### desired 1
    # pose.orientation.x = -0.703804
    # pose.orientation.y = -0.105292
    # pose.orientation.z = 0.0967695
    # pose.orientation.w = 0.695852 
    # ### 実機でorientationこの値が得られたから入力してるけどgazeboからは違う値出てくる
    
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
    
    # # *********************************************************************************
    # # 制約を定義
    # constraints = moveit_msgs.msg.Constraints()

    # # 各関節の制限を設定
    # joints_name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    # joints_goal = [1.678, -1.607, 1.867, -0.245, 1.383, -3.159]  # 目標角度（radians）
    # # tolerances = [0.1, 0.1, 0.1, 0.05, 0.125, 0.003]  # 許容範囲（radians）
    # tolerances = [1.0, 1.0, 2.0, 1.0, 1.0, 0.2]

    # for i in range(6):
    #     joint_constraint = moveit_msgs.msg.JointConstraint()
    #     joint_constraint.joint_name = joints_name[i]
    #     joint_constraint.position = joints_goal[i]
    #     joint_constraint.tolerance_above = tolerances[i]
    #     joint_constraint.tolerance_below = tolerances[i]
    #     joint_constraint.weight = 1.0
    #     constraints.joint_constraints.append(joint_constraint)

    # # 制約をMoveGroupに適用
    # move_group.set_path_constraints(constraints)
    # # *********************************************************************************

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
    
    # pose_data.append([pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    pose_data.append([current_pose.position.x, current_pose.position.y, current_pose.position.z])
    
    ### gazebo+rviz(moveit)のとき手先posiotionもorientationもずれてるので注意
    ### 実際には上のget_current_poseとget_current_jointで取得されてる値怪しい->jointの方はrobot_info.pyから取得してみたら大丈夫そうだった
    ### get_current_poseで取得される手先座標は怪しそう（距離データ取得くらいにしか使わない）
    ### vel_servo_sig.py内で距離データ取得するときjoint_planner_serverで取得したdesired_pose.csv読んでるけどこれ多分正しくない（gazeboから取得した値だから）
    ### vel_servo_sig.pyで距離デーら取得するとき目標位置直打ちしたほうがいいかも
    filename = './rndmth_result/random_theta_result.csv'
    filename2 = './rndmth_result/random_pose.csv'
    
    with open (filename, 'a') as f, open(filename2, 'a') as f2:
        writer = csv.writer(f)
        writer.writerows(joint_data)
        writer2 = csv.writer(f2)
        writer2.writerows(pose_data)
        
    # # *********************************************************************************
    # # 移動が終了したら制約をクリア
    # move_group.clear_path_constraints()
    # # *********************************************************************************
    
    result2 = EmptyResult()
    motion_server2.set_succeeded(result2)
    # moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node('random_motion_server')
    rospy.loginfo('Random motion sever started. Ready to serve.')
    motion_server2 = actionlib.SimpleActionServer('ugoke_random', EmptyAction, random_pose, False)
    motion_server2.start()
    rospy.spin()