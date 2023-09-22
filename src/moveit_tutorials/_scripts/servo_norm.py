#!/usr/bin/env python
#coding: UTF-8

###############グラフ出力
###servo.pyの出力をグラフにしてコメント消したバージョン

import rospy
import numpy as np
import cv2
import moveit_commander

import sys
from math import pi

import pandas as pd
# import csv

import math
from time import sleep

import matplotlib
matplotlib.use('Agg') ###error main thread is not in main loop の対策?
import matplotlib.pyplot as plt
# from matplotlib import animation

from moveit_tutorials.srv import GetCurrentImageData

from sklearn import preprocessing

import csv

pinv_int_mat_norm = np.empty((6,2073600))
move_group = None 
I_dsr_vec = np.empty((2073600, 1))
I_vec = np.empty((2073600, 1))
rsme = 100
lmbd = 0.07
t = 0.03 
nop = 2073600 #number of pixels
rsme_data = [] 
new_joint_goal_data = [] 

base_joint_data = []
shoulder_joint_data = []
elbow_joint_data = []
wrist1_joint_data = []
wrist2_joint_data = []
wrist3_joint_data = []

def readpickle():
    global pinv_int_mat_norm
    # readpickletime = time.time()
    pickledata = pd.read_pickle('./pinv_int_mat_pickle/pinv_int_mat_normalized.pickle') ###read normalized pinv_int_mat_norm
    # pinv_int_mat_norm = np.array ###これいる？？
    pinv_int_mat_norm = pickledata.values
    # endreadpickletime = time.time()
    # pickle_elpsdt = endreadpickletime - readpickletime
    # print('pickle elapsed time = %f' % pickle_elpsdt)
    return pinv_int_mat_norm

def gen_dsrimvec(): 
    global I_dsr_vec
    I_dsr = cv2.imread('./input_dsrim/kensyo_desired_image.png')
    I_dsr_gry = cv2.cvtColor(I_dsr, cv2.COLOR_BGR2GRAY)
    I_dsr_arr = np.array(I_dsr_gry, dtype = 'float64')
    I_dsr_cvec = I_dsr_arr.reshape(-1,1)
    I_dsr_vec = preprocessing.minmax_scale(I_dsr_cvec) ###normalize
    print('---I_dsr_vec dtype---')
    print(I_dsr_vec.dtype)
    return I_dsr_vec

def moveit_settings():
    global move_group
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander('manipulator')
    move_group.set_max_velocity_scaling_factor(value=0.1)
    move_group.set_max_acceleration_scaling_factor(value=0.1)
    print('moveit settings done')
    return move_group

def get_image(): ###tested in servo_test.py------------------------------------
    global I_vec
    # service_start_time = time.time()
    # print('waiting for image server')
    rospy.wait_for_service('kure')
    # print('image server found')
    image_client = rospy.ServiceProxy('kure', GetCurrentImageData)
    # print('kure request sent')
    response = image_client()
    I_vec_arr = np.array(response.current_image, dtype = 'float64') 
    I_cvec = I_vec_arr.reshape(-1,1)
    I_vec = preprocessing.minmax_scale(I_cvec) ###normalize
    # print('---I_vec dtype---')
    # print(I_vec.dtype)
    # service_end_time = time.time()
    # service_elapsed_time = service_end_time - service_start_time
    # print(service_elapsed_time)
    return response.current_image, I_vec

def servo():
    global pinv_int_mat_norm
    global move_group
    global lmbd
    global t
    global I_dsr_vec
    global I_vec
    global nop
    global rsme
    global rsme_data
    global new_joint_goal_data
    
    global base_joint_data
    global shoulder_joint_data
    global elbow_joint_data
    global wrist1_joint_data
    global wrist2_joint_data
    global wrist3_joint_data
    
    #画像偏差計算
    dI = I_dsr_vec - I_vec ###目標-現在！！！それぞれ正規化後
    # print(I_vec.dtype)
    # print(I_dsr_vec.dtype)
    # print(dI.dtype)
    dI2 = dI**2
    Isum = np.sum(dI2)
    rsme = math.sqrt(Isum / nop)
    print('rsme = %f' % rsme)
    
    #角速度計算
    d_vel = lmbd*(np.dot(pinv_int_mat_norm, dI)) ###制御則----------------
    # d_vel = -(lmbd*(np.dot(pinv_int_mat_norm, dI))) #+-反転
    
    #位置変化分計算
    d_joint_values = t*d_vel
    
    #set_joint_valuesに代入できる形に変換
    arr_d_joint_values = np.empty((6))
    arr_d_joint_values = d_joint_values[:,0]
    
    #現在ジョイント角に位置変化分を足す
    current_joint_values = np.array(move_group.get_current_joint_values())
    new_joint_goal = current_joint_values + arr_d_joint_values
    print(new_joint_goal)
    new_joint_goal_list = new_joint_goal.tolist()
    new_joint_goal_data.append(new_joint_goal_list)
    
    filename = './servo_data/loop_joint_goal_values_norm.csv'
    with open (filename, 'a') as f:
        writer = csv.writer(f)
        writer.writerows(new_joint_goal_data)
    
    #動かす
    move_group.set_joint_value_target(new_joint_goal)
    move_group.go(wait=True)
    move_group.stop() 
    ###forget_joint_values(name) <-nameに何入れるのか
    move_group.forget_joint_values('manipulator') ###保存された関節角を消去します。
    
    rsme_data.append(rsme)
    
    #各ジョイント角グラフのy軸とx軸
    base_joint_data.append(new_joint_goal[0])
    xbase = range(len(base_joint_data))
    shoulder_joint_data.append(new_joint_goal[1])
    xshoulder = range(len(shoulder_joint_data))
    elbow_joint_data.append(new_joint_goal[2])
    xelbow = range(len(elbow_joint_data))
    wrist1_joint_data.append(new_joint_goal[3])
    xwrist1 = range(len(wrist1_joint_data))
    wrist2_joint_data.append(new_joint_goal[4])
    xwrist2 = range(len(wrist2_joint_data))
    wrist3_joint_data.append(new_joint_goal[5])
    xwrist3 = range(len(wrist3_joint_data))
    
    #6つのグラフの配置
    fig = plt.figure(figsize = (10,10), facecolor = 'lightblue')
    ax1 = fig.add_subplot(3,2,1)
    ax2 = fig.add_subplot(3,2,2)
    ax3 = fig.add_subplot(3,2,3)
    ax4 = fig.add_subplot(3,2,4)
    ax5 = fig.add_subplot(3,2,5)
    ax6 = fig.add_subplot(3,2,6)
    
    #各ループごとの各ジョイント角プロット
    ax1.plot(xbase, base_joint_data, color = 'blue', label = '1')
    ax2.plot(xshoulder, shoulder_joint_data, color = 'blue', label = '2')
    ax3.plot(xelbow, elbow_joint_data, color = 'blue', label = '3')
    ax4.plot(xwrist1, wrist1_joint_data, color = 'blue', label = '4')
    ax5.plot(xwrist2, wrist2_joint_data, color = 'blue', label = '5')
    ax6.plot(xwrist3, wrist3_joint_data, color = 'blue', label = '6')
    
    # #目標角度をプロット
    ax1.axhline(y = 1.57, color = 'red')
    ax2.axhline(y = -1.22, color = 'red')
    ax3.axhline(y = 1.50, color = 'red')
    ax4.axhline(y = -1.82, color = 'red')
    ax5.axhline(y = -1.57, color = 'red')
    ax6.axhline(y = 0.0, color = 'red')
    
    #x軸のラベル
    ax1.set_xlabel('loop')
    ax2.set_xlabel('loop')
    ax3.set_xlabel('loop')
    ax4.set_xlabel('loop')
    ax5.set_xlabel('loop')
    ax6.set_xlabel('loop')
    
    #y軸のラベル
    ax1.set_ylabel('base joint')
    ax2.set_ylabel('shoulder joint')
    ax3.set_ylabel('elbow joint')
    ax4.set_ylabel('wrist1 joint')
    ax5.set_ylabel('wrist2 joint')
    ax6.set_ylabel('wrist3 joint')
    
    #縦軸横軸の最大値、最小値、目盛り自動設定のグラフ保存
    fig.savefig('./servo_data/joint_values_norm.png')
    
    #set_yticksに渡すための配列作り
    # base_axis_array = np.arange(np.min(base_joint_data), np.max(base_joint_data), 0.05)
    # shoulder_axis_array = np.arange(np.min(shoulder_joint_data), np.max(shoulder_joint_data), 0.05)
    # elbow_axis_array = np.arange(np.min(elbow_joint_data), np.max(elbow_joint_data), 0.05)
    # wrist1_axis_array = np.arange(np.min(wrist1_joint_data), np.max(wrist1_joint_data), 0.05)
    # wrist2_axis_array = np.arange(np.min(wrist2_joint_data), np.max(wrist2_joint_data), 0.05)
    # wrist3_axis_array = np.arange(np.min(wrist3_joint_data), np.max(wrist3_joint_data), 0.05)
    
    base_axis_array = np.arange(1.4, 1.8, 0.05)
    shoulder_axis_array = np.arange(-1.5, -1.1, 0.05)
    elbow_axis_array = np.arange(1.3, 1.7, 0.05)
    wrist1_axis_array = np.arange(-2.0, -1.6, 0.05)
    wrist2_axis_array = np.arange(-1.8, -1.4, 0.05)
    wrist3_axis_array = np.arange(-0.2, 0.2, 0.05)
    
    #グラフの目盛りのスケールを合わせる
    ax1.set_yticks(base_axis_array)
    ax2.set_yticks(shoulder_axis_array)
    ax3.set_yticks(elbow_axis_array)
    ax4.set_yticks(wrist1_axis_array)
    ax5.set_yticks(wrist2_axis_array)
    ax6.set_yticks(wrist3_axis_array)
    
    #スケールを合わせたグラフを保存
    fig.savefig('./servo_data/joint_values_norm_scaled.png')
        
    #generate graph
    fig_rsme = plt.figure()
    plt.xlabel('loop')
    plt.ylabel('RSME')
    loop = range(len(rsme_data))
    plt.plot(loop, rsme_data, 'b-')
    fig_rsme.savefig('./servo_data/rsme_norm.png')
    return rsme, rsme_data
    # return rsme, rsme_data, new_joint_goal_data
    
def main():
    while not rospy.is_shutdown():
        print('===================')
        global rsme 
        # loop_start_time = time.time()
        if rsme > 0.04:
            get_image()
            # start_image_rate = time.time()
            servo()
        else:
            print('done')
            break
        # loop_end_time = time.time()
        # image_rate = loop_end_time - start_image_rate
        # loop_elapsed_time = loop_end_time - loop_start_time
        # print(image_rate)
        # print(loop_elapsed_time)
    
if __name__ == '__main__':
    try:
        rospy.init_node('servo')
        rospy.loginfo('servo node started')
        readpickle()
        moveit_settings()
        gen_dsrimvec()
        main()
    except rospy.ROSInterruptException:
        pass
    