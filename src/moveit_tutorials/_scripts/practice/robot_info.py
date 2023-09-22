#!/usr/bin/env python
# coding: UTF-8
import sys
import moveit_commander
import rospy

# main
def main():
    # MoveitCommander initialize
    moveit_commander.roscpp_initialize(sys.argv)

    # init node
    rospy.init_node('robot_info')

    # RobotCommander instantiation
    robot = moveit_commander.RobotCommander()

    # robot information
    print('==Robot Info==')
    print('[ group_names ]', robot.get_group_names())
    #get_current_state()でvelocityまでわかってるみたいだけど速度入力は可能?
    print('[ current_state ] ', robot.get_current_state())

    # MoveGroupCommander instantiation
    move_group = moveit_commander.MoveGroupCommander('manipulator')

    # group informaion
    print('==Group Info==')
    print('[ name ] ', move_group.get_name())
    print('[ planning_frame ] ', move_group.get_planning_frame())
    print('[ interface_description ] ', move_group.get_interface_description())

    # joint information
    print('==Joint Info==')
    print('[ active_joints ] ', move_group.get_active_joints())
    print('[ joints ] ', move_group.get_joints())
    print('[ current_joint_values ] ', move_group.get_current_joint_values())

    # end effector information
    print('==EndEffector Info==')
    print('[ has_end_effector_link ] ', move_group.has_end_effector_link())
    print('[ end_effector_link ] ', move_group.get_end_effector_link())
    print('[ current_pose ] ', move_group.get_current_pose())
    print('[ current_rpy ] ', move_group.get_current_rpy())


if __name__ == '__main__':
    main()
