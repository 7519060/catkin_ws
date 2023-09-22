#!/usr/bin/env python
#coding: UTF-8

import rospy
import numpy as np
import cv2
import moveit_commander
import sys
import pandas as pd
import math

import matplotlib
matplotlib.use('Agg') ###error main thread is not in main loop の対策?
import matplotlib.pyplot as plt

import csv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# import signal----------------------------------------------------

#正規化なし、画像ヤコビアンは型をdoubleに統一して計算

#動かしてる最中にジョイント角とそのときの画像を保存

pinv_int_mat_double = np.empty((6,2073600))
move_group = None
I_dsr_vec = np.empty((2073600, 1))
rsme = 100
lmbd = 0.7
t = 0.03 
nop = 2073600 #number of pixels
# new_joint_goal = np.empty((6,1))
rsme_data = [] 
new_joint_goal_data = [] 

base_joint_data = []
shoulder_joint_data = []
elbow_joint_data = []
wrist1_joint_data = []
wrist2_joint_data = []
wrist3_joint_data = []

def readpickle():
    global pinv_int_mat_double
    pickledata = pd.read_pickle('./pinv_int_mat_pickle/pinv_int_mat_double.pickle')
    pinv_int_mat_double = pickledata.values
    return pinv_int_mat_double

def gen_dsrimvec(): 
    global I_dsr_vec
    I_dsr = cv2.imread('./input_dsrim/kensyo_desired_image.png')
    I_dsr_gry = cv2.cvtColor(I_dsr, cv2.COLOR_BGR2GRAY)
    I_dsr_arr = np.array(I_dsr_gry, dtype = 'float64')
    I_dsr_vec = I_dsr_arr.reshape(-1,1)
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

def servo():
    global pinv_int_mat_double
    global move_group
    global lmbd
    global t
    global I_dsr_vec
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
    
    #subscribe image raw with rospy.wait_for_message
    print('wait')
    image_raw = rospy.wait_for_message('/camera/color/image_raw', Image)
    print('received')
    bridge = CvBridge()
    bgr = bridge.imgmsg_to_cv2(image_raw, 'bgr8') ###この時点でnumpy.arrayになってる1080,1920,3
    gry = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gry2 = np.array(gry, dtype = 'float64')
    I_vec = gry2.reshape(-1,1)
    
    #画像偏差計算
    dI = I_dsr_vec - I_vec ###目標-現在！！！
    dI2 = dI**2
    Isum = np.sum(dI2)
    rsme = math.sqrt(Isum / nop)
    print('rsme = %f' % rsme)

    #角速度計算
    d_vel = lmbd*(np.dot(pinv_int_mat_double, dI)) 
    # print(d_vel)
    
    #位置変化分計算
    d_joint_values = t*d_vel
    # print(d_joint_values) 
    
    #set_joint_valuesに代入できる形に変換
    arr_d_joint_values = np.empty((6))
    arr_d_joint_values = d_joint_values[:,0]
    
    #現在ジョイント角に位置変化分を足す
    ###初期化的なやつ必要？？？movegroup.clear joint values みたいなのあった気がする
    current_joint_values = np.array(move_group.get_current_joint_values())
    # print(current_joint_values.shape) #(6,)の形になってた
    new_joint_goal = current_joint_values + arr_d_joint_values
    print(new_joint_goal)
    new_joint_goal_list = new_joint_goal.tolist()
    new_joint_goal_data.append(new_joint_goal_list)
    
    #動かす
    move_group.set_joint_value_target(new_joint_goal)
    move_group.go(wait=True)
    move_group.stop() 
    move_group.forget_joint_values('manipulator') ###保存された関節角を消去します。
    
    rsme_data.append(rsme)
    
    #各ジョイント角グラフのy軸とx軸
    base_joint_data.append(new_joint_goal[0])
    shoulder_joint_data.append(new_joint_goal[1])
    elbow_joint_data.append(new_joint_goal[2])
    wrist1_joint_data.append(new_joint_goal[3])
    wrist2_joint_data.append(new_joint_goal[4])
    wrist3_joint_data.append(new_joint_goal[5])
    x = range(len(base_joint_data))
    
    #6つのグラフの配置
    fig = plt.figure(figsize = (10,10), facecolor = 'lightblue')
    ax1 = fig.add_subplot(3,2,1)
    ax2 = fig.add_subplot(3,2,2)
    ax3 = fig.add_subplot(3,2,3)
    ax4 = fig.add_subplot(3,2,4)
    ax5 = fig.add_subplot(3,2,5)
    ax6 = fig.add_subplot(3,2,6)
    
    #各ループごとの各ジョイント角プロット
    ax1.plot(x, base_joint_data, color = 'blue', label = '1')
    ax2.plot(x, shoulder_joint_data, color = 'blue', label = '2')
    ax3.plot(x, elbow_joint_data, color = 'blue', label = '3')
    ax4.plot(x, wrist1_joint_data, color = 'blue', label = '4')
    ax5.plot(x, wrist2_joint_data, color = 'blue', label = '5')
    ax6.plot(x, wrist3_joint_data, color = 'blue', label = '6')
    
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
    
    #重なり解消
    plt.tight_layout()
    
    #縦軸横軸の最大値、最小値、目盛り自動設定のグラフ保存
    fig.savefig('./servo_data/joint_values_double.png')
    
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
    fig.savefig('./servo_data/joint_values_double_scaled.png')

    #generate graph
    fig_rsme = plt.figure()
    plt.xlabel('loop')
    plt.ylabel('RSME')
    loop = range(len(rsme_data))
    plt.plot(loop, rsme_data, 'b-')
    fig_rsme.savefig('./servo_data/rsme_double.png')
    # return rsme, rsme_data
    return rsme, rsme_data, new_joint_goal_data
    
def main():
    filename = './servo_data/loop_joint_goal_values_double.csv'
    i = 1
    with open (filename, 'w') as f:
        while not rospy.is_shutdown():
            print('===================')
            global rsme 
            if rsme > 10:
                print('loop %d' % i)
                i += 1
                servo()
                writer = csv.writer(f)
                writer.writerows(new_joint_goal_data)
            else:
                print('done')
                break

    
if __name__ == '__main__':
    try:
        # signal.signal(signal.SIGINT, handler)------------------------
        rospy.init_node('servo')
        rospy.loginfo('servo node started')
        readpickle()
        moveit_settings()
        gen_dsrimvec()
        main()
    except rospy.ROSInterruptException:
        pass
    