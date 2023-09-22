#!/usr/bin/env python
#coding: UTF-8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from moveit_tutorials.srv import GetCurrentImageData, GetCurrentImageDataResponse
# import numpy as np

# import datetime

# from sklearn import preprocessing

# from skimage import color

#imgmsg to cv2でnumpy arrayになっちゃうのでreshapeまで行う
#serviceで渡すときはuint8のまま

# bgr = None
gry_cvec = []
    
def henkan(msg):
    global gry_cvec
    # global bgr
    bridge = CvBridge()
    
    # for realsense camera
    bgr = bridge.imgmsg_to_cv2(msg, 'bgr8') ###この時点でnumpy.arrayになってる1080,1920,3
    gry = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    
    ###for ids camera
    # gry = bridge.imgmsg_to_cv2(msg, 'mono8')
    
    # # rgb = bridge.imgmsg_to_cv2(msg, 'rgb8') ###これでmatlab im2grayと同じ処理かと思ったけど違うぽい
    # # gry = color.rgb2gray(rgb) ###max調べたら1じゃなかたったので最大255最小0?
    gry_cvec = gry.reshape(-1,1)
    # print(type(gry_cvec)) 
    # print(gry_cvec.shape)
    # print(gry_cvec.dtype)
    # gry_norm = preprocessing.minmax_scale(gry_cvec) ###ここでfloat64になるwarnings.warn(msg, DataConversionWarning)の原因?
    # print(max(gry_norm)) ###ちゃんと1になってるminも0
    # print(min(gry_norm))
    # print(type(gry_norm)) 
    # print(gry_norm.shape)
    # print(gry_norm.dtype)
    return gry_cvec
    # return gry_cvec, bgr

#server の中の処理は減らしてすぐにresponse送れるようにする
def send_image(req): ###Empty requestに対してはreq不使用のまま(?)
    global gry_cvec
    # global bgr
    print('sent image')
    # now = datetime.datetime.now()
    # image_name = './servo_data/' + now.strftime('%Y%m%d_%H%M%S') + '_loop_image.png'
    # cv2.imwrite(image_name, bgr)
    # print('saved loop image')
    # cv2.waitKey(1)
    
    # print('-----') ###gry_cvecのtype, shape, dtype変わらずに渡ってる
    # print(type(gry_cvec))
    # print(gry_cvec.shape)
    # print(gry_cvec.dtype)
    # print('-----')
    return GetCurrentImageDataResponse(current_image = gry_cvec)

def sub_image():
    rospy.loginfo('ready to send image to client')
    rospy.Subscriber('/camera/color/image_raw', Image, henkan)
    # rospy.Subscriber('/camera/image_raw', Image, henkan) #for UI camera
    rospy.Service('kure', GetCurrentImageData, send_image)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        rospy.init_node('image_server')
        sub_image()
    except rospy.ROSInterruptException:
        pass