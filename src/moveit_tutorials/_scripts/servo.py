#!/usr/bin/env python
#coding: UTF-8

###############txtファイルを出力

import rospy
# from std_msgs.msg import Float64MultiArray
# from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge
import moveit_commander

import sys
from math import pi

import pandas as pd
import csv
import glob
import time

import math
from time import sleep

import matplotlib.pyplot as plt
from matplotlib import animation

pinv_int_mat = None
move_group = None
I_dsr_vec = None
# start_servo_time = None
# Ith = 500000000 
rsme = 100
lmbd = 0.01
t = 0.03 #(sec) same as image_raw publish rate
nop = 2073600 ###number of pixels : 1080 x 1920 (grayscale) 
# rsme_data = [] ###ループの中で要素が増えていったものが生成される
# new_joint_goal_data = [] ###ループごとに要素が増えていっちゃう

# def readcsv():
#     global pinv_int_mat
#     readcsvtime = time.time()
#     csvdata = pd.read_csv('./pinv_int_mat/pinv_int_mat.csv', header=None)
#     # print('csvyometa')
#     pinv_int_mat = np.array
#     pinv_int_mat = csvdata.values
#     endreadcsvtime = time.time()
#     csv_elpsdt = endreadcsvtime - readcsvtime
#     print('csv elapsed time = %f' % csv_elpsdt)
#     print('read csv file values')
#     return pinv_int_mat

def readpickle():
    global pinv_int_mat
    readpickletime = time.time()
    pickledata = pd.read_pickle('./pinv_int_mat_pickle/pinv_int_mat.pickle')
    pinv_int_mat = np.array
    pinv_int_mat = pickledata.values
    endreadpickletime = time.time()
    pickle_elpsdt = endreadpickletime - readpickletime
    print('pickle elapsed time = %f' % pickle_elpsdt)
    return pinv_int_mat

def gen_dsrimvec():
    global I_dsr_vec
    #generating desired image column vector
    dsrim_path = './input_dsrim/*_image.png'
    # print('set dsrim path')
    dsrim_name = glob.glob(dsrim_path)
    I_dsr = cv2.imread(str(dsrim_name[0]), cv2.IMREAD_GRAYSCALE)
    # print('dssr im yometa')
    I_dsr_vec = I_dsr.reshape(-1,1)
    print('dsrimvec generated')
    return I_dsr_vec

def moveit_settings():
    global move_group
    global start_servo_time
    print('moveit settings')
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander('manipulator')
    move_group.set_max_velocity_scaling_factor(value=0.1)
    move_group.set_max_acceleration_scaling_factor(value=0.1)
    print('moveit settings done')
    # global start_servo_time
    # start_servo_time = time.time()
    return move_group
    # return move_group, start_servo_time

def servo(data):
    global pinv_int_mat
    global move_group
    # global Ith
    global lmbd
    global t
    global I_dsr_vec
    global nop
    global rsme
    # global start_servo_time
    # global rsme_data
    # global new_joint_goal_data
    
    
    # print('-----subscribed-----')
    # print('Ith = %d' % Ith)
    # print( rsme = %f' % rsme)
    
    # #generating desired image column vector　#-->gen_dsrimvecへ移動
    # dsrim_path = './input_dsrim/*_image.png'
    # print('set dsrim path')
    # dsrim_name = glob.glob(dsrim_path)
    # I_dsr = cv2.imread(str(dsrim_name[0]), cv2.IMREAD_GRAYSCALE)
    # print('dssr im yometa')
    # I_dsr_vec = I_dsr.reshape(-1,1)
    # print('dsrimvec generated')
    
    # #for moveit setups #-->moveit_settingsへ移動
    # print('moveit settei')
    # moveit_commander.roscpp_initialize(sys.argv)
    # move_group = moveit_commander.MoveGroupCommander('manipulator')
    # move_group.set_max_velocity_scaling_factor(value = 0.1)
    # move_group.set_max_acceleration_scaling_factor(value = 0.1)
    # print('moveit settei owari')

    # #start timer for graph
    # start_servo_time = time.time() ###ここに書いたら毎ループスタート変わっちゃう
    
    #generating live image column vector
    # print('imageraw conversion started') 
    bridge = CvBridge()
    bgr = bridge.imgmsg_to_cv2(data, 'bgr8')
    # print('bgr shape:')
    # print(type(bgr)) ###type 'numpy.ndarray'
    # print(bgr.shape) ###(1080,1920,3)
    gry = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    # print('gry shape:')
    # print(gry.shape) ###(1080,1920)
    I_vec = gry.reshape(-1,1)
    # print('image column vector shape:')
    # print(I_vec.shape) ###(2073600,1)
    
    #角速度計算
    # print('keisan started')
    dI = I_dsr_vec - I_vec
    Ith = np.sum(dI)
    # print('Ith = %d' % Ith)
    rsme = math.sqrt(Ith / nop)
    print('rsme = %f' % rsme)
    # print('pseudo inverse interaction matrix shape:')
    # print(type(pinv_int_mat)) ###type 'numpy.ndarray'
    # print(pinv_int_mat.shape) ###(6,2073600)
    d_vel = -lmbd*(np.dot(pinv_int_mat, dI)) #角速度
    # print('d_vel shape:')
    # print(type(d_vel)) ###type 'numpy.ndarray'
    # print(d_vel.shape) ###(6,1)
    # print(d_vel)
    
    #位置変化分計算
    d_joint_values = t*d_vel
    # print('d_joint_values type, shape and values:')
    # print(type(d_joint_values)) ###type 'numpy.ndarray'
    # print(d_joint_values.shape) ###(6,1)
    # print(d_joint_values) ###ちゃんとd_vel各要素に時間0.03sかかってた
    
    #set_joint_valuesに代入できる形に変換
    arr_d_joint_values = np.empty((6))
    arr_d_joint_values = d_joint_values[:,0]
    # print('arr_d_joint_values type, shape and values')
    # print(type(arr_d_joint_values)) ###type 'numpy.ndarray'
    # print(arr_d_joint_values.shape) ###(6,)
    # print('arr_d_joint_values') #<---
    # print(arr_d_joint_values) #<---
    #現在ジョイント角に位置変化分を足す
    current_joint_values = np.array(move_group.get_current_joint_values())
    new_joint_goal = current_joint_values + arr_d_joint_values
    # print('new_joint_goal type, shape, and values')
    # print(type(new_joint_goal)) ###type 'numpy.ndarray'
    # print(new_joint_goal.shape) ###(6,)
    # print('new_joint_goal values')
    # print(new_joint_goal)
    # print(new_joint_goal[0])
    
    #動かす
    move_group.set_joint_value_target(new_joint_goal)
    move_group.go(wait=True)
    move_group.stop() #動きが残っていないことを保証
    # sleep(1)
    # print('moved')
    
    # #set finished time
    # end_servo_time = time.time()
    
    # #record time
    # servo_elpsd_time = end_servo_time - start_servo_time
    # TIME_data = [] ###大文字TIMEにしたらなんかあかん
    # TIME_data.append(servo_elpsd_time)
    
    #write rsme result in txt file
    filename = './servo_data/rsme_result.txt'
    # new_joint_goal_data = []
    rsme_data = []
    rsme_data.append(rsme)
    # rsme_data.extend([rsme])
    # rsme_data = np.empty(0)
    # rsme_data = np.append(rsme_data, rsme)
    with open (filename, 'a') as f:
        # rsme_data.append(rsme)
        f.write(str(rsme_data))
    filename2 = './servo_data/joint_values.txt'
    new_joint_goal_data = []
    new_joint_goal_list = new_joint_goal.tolist()
    # print(type(new_joint_goal)) ###numpy.ndarray
    # print(type(new_joint_goal_list)) ###list
    new_joint_goal_data.append(new_joint_goal_list)
    # new_joint_goal_data.extend([new_joint_goal])
    with open (filename2, 'a') as f2:
        f2.write(str(new_joint_goal_data))
        
    #generate graph
    # plt.plot rsme_data, TIME_data, 'b-')
    # plt.savefig('./servo_data rsme_t.png')
    # return Ith
    return rsme, rsme_data, new_joint_goal_data
    
def threshold(data): #servoの前に条件分岐
    global rsme
    if rsme > 5.2:
        servo(data)
    else:
        print('done')
        pass
    return 0
    
def main():
    print('hi')
    # global Ith
    global rsme
    rospy.Subscriber('/camera/color/image_raw', Image, threshold)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        rospy.init_node('servo')
        rospy.loginfo('servo node started')
        # readcsv()
        readpickle()
        moveit_settings()
        gen_dsrimvec()
        main()
    except rospy.ROSInterruptException:
        pass


# # rospy.init_node('servo')
# # pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
# # msg = Float64MultiArray()
# # msg.data =[0, 0, 0, 0, 0, 0.2]
# # msg.layout.data_offset = 1 #nanikore
# # rate = rospy.Rate(10)

# # while rospy.is_shutdown():
# #     pub.publish(msg)
# #     rate.sleep()
#topicで速度制御これでできるかもなコード
# pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
# # pub = rospy.Publisher('/vel_based_pos_traj_controller', Float64MultiArray, queue_size=10)
# msg = Float64MultiArray()
# msg.data =[0, 0, 0, 0.5, 0.3, 0.2]
# msg.layout.data_offset = 1 #nanikore
# rate = rospy.Rate(1)
# # for i in range(5):
# #     print('for bun')
# #     pub.publish(msg)
# #     print('published')
# #     rate.sleep()
# while not rospy.is_shutdown():
#     print('while not haitta')
#     pub.publish(msg)
#     rate.sleep()


