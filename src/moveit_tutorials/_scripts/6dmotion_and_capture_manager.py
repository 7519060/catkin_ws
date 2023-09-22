#!/usr/bin/env python
# coding: UTF-8

import rospy
import actionlib
from moveit_tutorials.msg import ValidJointsGoal, ValidJointsAction
from moveit_tutorials.msg import EmptyAction, EmptyGoal
import csv
from time import sleep
from std_srvs.srv import Empty

def motion_command():
    # motion_client = actionlib.SimpleActionClient('ugoke', EmptyAction)
    motion_goal = EmptyGoal()
    print('waiting for motion server')
    motion_client.wait_for_server()
    motion_client.send_goal(motion_goal)
    motion_client.wait_for_result()
    print('got result from motion node')
    motion_result = motion_client.get_result()
    
def random_motion_command():
    # random_motion_client = actionlib.SimpleActionClient('ugoke_random', ValidJointsAction)
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
    # capture_client = rospy.ServiceProxy('tore', Empty)
    print('caoture request sent')
    responce = capture_client()
    print('got responce from capture node')

# def main():
#     # Initialize the client
#     rospy.init_node('motion_and_capture_manager')
#     client = actionlib.SimpleActionClient('ugoke_random', ValidJointsAction)
#     client.wait_for_server()

#     # Read the CSV file
#     with open('./rndmth_result/random_theta_result.csv', 'r') as f:
#         reader = csv.reader(f)
#         for i, row in enumerate(reader):
#             # Create goal from row data
#             # random_motion_goal = ValidJointsGoal()
#             random_motion_goal.valid_joint_values = [float(x) for x in row]
#             client.send_goal(random_motion_goal)

#             # Wait for result
#             # client.wait_for_result(rospy.Duration.from_sec(5.0))
#             client.wait_for_result()
#             result = client.get_result()

if __name__ == '__main__':
    try:
        # main()
        rospy.init_node('motion_and_capture_manager')
        motion_client = actionlib.SimpleActionClient('ugoke', EmptyAction)
        # motion_goal = EmptyGoal()
        
        random_motion_client = actionlib.SimpleActionClient('ugoke_random', ValidJointsAction)
        # random_motion_goal = ValidJointsGoal()
        
        capture_client = rospy.ServiceProxy('tore', Empty)
        
        motion_command()
        sleep(0.5)
        capture_command()
        with open('./rndmth_result/random_theta_result.csv', 'r') as f:
            reader = csv.reader(f)
            for i, row in enumerate(reader):
                print('i =')
                print(i)
                random_motion_command()
                sleep(0.5)
                capture_command()
    except rospy.ROSInterruptException:
        pass
