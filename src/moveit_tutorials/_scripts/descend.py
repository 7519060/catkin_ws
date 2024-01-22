#!/usr/bin/env python
# coding: UTF-8

import sys
import rospy
import moveit_commander
from math import pi

def zdescend():
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_max_velocity_scaling_factor(value=0.08)
    move_group.set_max_acceleration_scaling_factor(value=0.08) ###magnetのときこのスピードで抜けたwas0.01
 
    # Get current pose
    # このときorientationも取得されるのでz方向の制御量だけ入力すればいい
    current_pose = move_group.get_current_pose().pose
    print (current_pose)

    # Update Z coordinate
    target_pose = current_pose
    
    target_pose.position.z = 0.235 ###一回目の中継地点
    print('chukei')
    print(target_pose)
    move_group.set_pose_target(target_pose)
    move_group.go(wait=True)
    
    target_pose.position.z = 0.225 ###二回目の中継地点
    print('chukei')
    print(target_pose)
    move_group.set_pose_target(target_pose)
    move_group.go(wait=True)
    
    # target_pose.position.z = 0.223 ###without magnet
    target_pose.position.z = 0.211 ###ボルトの頭当たるくらい
    print('sasarucyokuzen')
    print(target_pose)
    move_group.set_pose_target(target_pose)
    move_group.go(wait=True)
    ### taiki
    print('ascended to z=22.4mm')
    
    # move_group.set_max_velocity_scaling_factor(value=0.008) #0.13
    # move_group.set_max_acceleration_scaling_factor(value=0.08) #0.13
    # # target_pose.position.z = 0.2175
    # target_pose.position.z = 0.208 ###押し込んでる0.2060was 0.21(1127was)0.2095(1208was)
    # print('new target pose')
    # print(target_pose)
    # move_group.set_pose_target(target_pose)
    # move_group.go(wait=True)
    # print('descended')
    
    # target_pose.position.z = 0.250
    # print('new target pose')
    # print(target_pose)
    # move_group.set_pose_target(target_pose)
    # move_group.go(wait=True)
    # print('descended')

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    rospy.init_node('descend')
    try:
        zdescend()
    except rospy.ROSInterruptException:
        pass
