#!/usr/bin/env python
# coding: UTF-8

import rospy
import actionlib
from moveit_tutorials.msg import ValidJointsGoal, ValidJointsAction
from moveit_tutorials.msg import EmptyAction, EmptyGoal
import csv
from time import sleep
from std_srvs.srv import Empty
from moveit_tutorials.srv import GetCurrentJointPos
from std_msgs.msg import Int32
from moveit_tutorials.msg import FlagAction, FlagGoal

joint_pos_values_data = []

def motion_command():
    motion_goal = EmptyGoal()
    print('waiting for motion server')
    motion_client.wait_for_server()
    motion_client.send_goal(motion_goal)
    motion_client.wait_for_result()
    print('got result from motion node')
    motion_result = motion_client.get_result()
    
def random_motion_command():
    random_motion_goal = ValidJointsGoal()
    print('waiting for random motion server')
    random_motion_client.wait_for_server()
    random_motion_goal.valid_joint_values = [float(x) for x in row]
    print(random_motion_goal)
    random_motion_client.send_goal(random_motion_goal)
    random_motion_client.wait_for_result()
    print('got result from random motion node')
    random_motion_result = random_motion_client.get_result()
    
def capture_command():
    rospy.wait_for_service('tore')
    print('caoture request sent')
    responce = capture_client()
    print('got responce from capture node')
    
def joint_position_command():
    global joint_pos_values_data
    rospy.wait_for_service('getpos') ### joint position = joint angle
    respos = getang_client()
    
    current_base_pos = respos.current_joint_pos[2] ###[2]がなんでbase joint なのかよくわからん
    current_shoulder_pos = respos.current_joint_pos[1]
    current_elbow_pos = respos.current_joint_pos[0] ###[0]がなんでelbow jointかよくわからん
    current_wrist1_pos = respos.current_joint_pos[3]
    current_wrist2_pos = respos.current_joint_pos[4]
    current_wrist3_pos = respos.current_joint_pos[5]
    joint_pos_values_data.append([current_base_pos, current_shoulder_pos, current_elbow_pos, current_wrist1_pos, current_wrist2_pos, current_wrist3_pos])
    return joint_pos_values_data
    
# def change_orientation_command(flag):
#     flag_goal = FlagGoal()
#     change_ori_client.wait_for_server()
#     flag_goal.flag = flag
#     change_ori_client.send_goal(flag_goal)
#     change_ori_client.wait_for_result()

if __name__ == '__main__':
    try:
        rospy.init_node('motion_and_capture_manager')
        motion_client = actionlib.SimpleActionClient('ugoke', EmptyAction)
        
        random_motion_client = actionlib.SimpleActionClient('ugoke_random', ValidJointsAction)
        
        capture_client = rospy.ServiceProxy('tore', Empty)
        
        getang_client = rospy.ServiceProxy('getpos', GetCurrentJointPos)
        
        # change_ori_client = actionlib.SimpleActionClient('flag', FlagAction)
        
        motion_command()
        sleep(0.5)
        capture_command()
        with open('./rndmth_result/random_theta_result.csv', 'r') as f:
            reader = csv.reader(f)
            for i, row in enumerate(reader):
                print('i =', i)
                random_motion_command()
                sleep(0.5)
                capture_command()
                joint_position_command()
                sleep(0.5)
                
                # if i % 2 == 0:
                #     change_orientation_command(0)
                #     sleep(0.5)
                #     capture_command()
                #     joint_position_command()
                #     sleep(0.5)
                #     print('yaw +0.1')
                    
                # else:
                #     change_orientation_command(1)
                #     sleep(0.5)
                #     capture_command()
                #     joint_position_command()
                #     sleep(0.5)
                #     print('yaw -0.1')
                
        with open('./rndmth_result/dataset_random_joint_angle.csv', 'w')as f2:
            writer2 = csv.writer(f2)
            writer2.writerow(joint_pos_values_data)

    except rospy.ROSInterruptException:
        pass
