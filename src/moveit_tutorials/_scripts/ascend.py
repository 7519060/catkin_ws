#!/usr/bin/env python
# coding: UTF-8

import sys
import rospy
import moveit_commander
from math import pi

def zdescend():
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("manipulator")
    move_group.set_max_velocity_scaling_factor(value=0.05)
    move_group.set_max_acceleration_scaling_factor(value=0.05)
 
    # Get current pose
    # このときorientationも取得されるのでz方向の制御量だけ入力すればいい
    current_pose = move_group.get_current_pose().pose
    print (current_pose)

    # Update Z coordinate
    target_pose = current_pose
    target_pose.position.z = 0.240  # Set Z coordinate to 300 mm (converted to meters as MoveIt uses meters)
    print(target_pose)

    move_group.set_pose_target(target_pose)
    move_group.go(wait=True)
    print('ascended')

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    rospy.init_node('descend')
    try:
        zdescend()
    except rospy.ROSInterruptException:
        pass
