#!/usr/bin/env python
#coding: UTF-8

import rospy
from moveit_tutorials.srv import GettfPose
# from geometry_msgs.msg import Point

# def request_pose():
#     rospy.init_node('pose_client')
#     rospy.wait_for_service('get_pose')
    
#     try:
#         get_pose = rospy.ServiceProxy('get_pose', GettfPose)
#         response = get_pose()
#         return response.position
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

# if __name__ == "__main__":
#     point = request_pose()
#     if point is not None:
#         print("wrist_3_link position in base_link coordinates: x=%s, y=%s, z=%s"%(point.x, point.y, point.z))
#     else:
#         print("Failed to get the position")
        
rospy.init_node('pose_client')
rospy.wait_for_service('getpose')
get_pose = rospy.ServiceProxy('getpose', GettfPose)
response = get_pose()
print(response.trans)