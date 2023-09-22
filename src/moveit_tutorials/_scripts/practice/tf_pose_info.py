#!/usr/bin/env python
#coding: UTF-8

import rospy
import tf

#base_link からwrist_3_linkへの座標変換
#tfのテスト
#手先の三次元座標を返してくれる(x,y,z,rx,ry,rz)

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0) 
    while not rospy.is_shutdown():
        try:
            # The listener returns a tuple (translation, rotation) representing the position and orientation of wrist_3_link with respect to base_link
            #lookupTransformでsubscribeしてくれてる
            (trans, rot) = listener.lookupTransform('/base_link', '/wrist_3_link', rospy.Time(0))
            rospy.loginfo("Translation: %s" % str(trans))
            rospy.loginfo("Rotation: %s" % str(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()
