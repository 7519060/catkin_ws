#!/usr/bin/env python
#coding: UTF-8

import rospy
import tf
from moveit_tutorials.srv import GettfPose, GettfPoseResponse
# from geometry_msgs.msg import Point

#tfで座標変換して手先の三次元座標を返すサーバーノード
#tfのテスト

def handle_get_pose(req):
    try:
        # listener = tf.TransformListener()
        (trans, rot) = listener.lookupTransform('/base_link', '/wrist_3_link', rospy.Time(0))
        print(str(trans))
        print(type(trans))
        print(str(rot))
        print(type(rot))
        return GettfPoseResponse(trans)
        # return Point(*trans)
    # except (tf.LookupException):
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        ### tf.LookupExceptionのエラー出てるっぽい
        ### tf_pose_info.pyではこのエラー出てないからまじで謎
        rospy.logerr("Cannot get transform")
        return GettfPoseResponse([])

# def pose_server():
#     rospy.init_node('pose_server')
#     listener = tf.TransformListener()
#     rospy.Service('getpose', GettfPose, handle_get_pose)
#     rospy.loginfo("Ready to get pose.")
#     rospy.spin()

if __name__ == "__main__":
    # pose_server()
    rospy.init_node('pose_server')
    listener = tf.TransformListener()
    rospy.Service('getpose', GettfPose, handle_get_pose)
    rospy.loginfo("Ready to get pose.")
    rospy.spin()
