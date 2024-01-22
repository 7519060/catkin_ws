#!/usr/bin/env python
#coding: UTF-8

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rospy
import numpy as np
import cv2
import pandas as pd
import math
import time
import csv
from sensor_msgs.msg import Image
# from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import signal
import sys

import atexit

#for velocity control
from std_msgs.msg import Float64MultiArray
#for joint states servicecall
from moveit_tutorials.srv import GetCurrentJointVel
#for pose servicecall
from moveit_tutorials.srv import GettfPose
#for switch controller
from controller_manager_msgs.srv import SwitchController

#number of pixels
# nop = 1105920 ###UI camera 1080*1024
nop = 2073600 ###realsense 1920*1080
pinv_int_mat_double = np.empty((6,nop))
I_dsr_vec = np.empty((nop, 1))
lmbd = 0.8

rmseth = 35

time_series = []
rmse_data = []

base_joint_data = []
shoulder_joint_data = []
elbow_joint_data = []
wrist1_joint_data = []
wrist2_joint_data = []
wrist3_joint_data = []

joint_vel_values_data = []

pose_data = []
dist_data = []

def switch_controller(start_controllers, stop_controllers):
    switch_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
    resp = switch_service(start_controllers=start_controllers, 
                          stop_controllers=stop_controllers,
                          strictness=2,
                          start_asap=False,
                          timeout=0.0)
    print(resp.ok)

def switch_to_joint_group_vel():
    switch_controller(['joint_group_vel_controller'], ['scaled_pos_joint_traj_controller'])

def switch_to_scaled_pos():
    switch_controller(['scaled_pos_joint_traj_controller'], ['joint_group_vel_controller'])


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
    return I_dsr_vec

def signal_handler(sig, frame):
    global time_series
    global rmse_data
    
    global base_joint_data
    global shoulder_joint_data
    global elbow_joint_data
    global wrist1_joint_data
    global wrist2_joint_data
    global wrist3_joint_data
    
    global joint_vel_values_data
    
    global pose_data
    global dist_data
    
    vel_input.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    vel_pub.publish(vel_input)
    print('stop command sent')
    
    filename = './servo_data/ex2/loop_rmse_data.csv'
    filename2 = './servo_data/ex2/joint_vel_data.csv'
    filename3 = './servo_data/ex2/pose_data.csv'
    with open (filename, 'w') as f, open(filename2, 'w')as f2, open(filename3, 'w')as f3:
        writer = csv.writer(f)
        writer.writerow(rmse_data)
        writer2 = csv.writer(f2)
        writer2.writerow(joint_vel_values_data)
        writer3 = csv.writer(f3)
        writer3.writerow(pose_data)

    #generate graph
    fig_rmse = plt.figure()
    # plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 15
    plt.xlabel('Time[s]', fontsize=18)
    plt.ylabel('RMSE', fontsize=18)
    plt.plot(time_series, rmse_data, 'b-')
    plt.grid()
    fig_rmse.savefig('./servo_data/ex2/rmse_double.png')
    # fig_rmse.savefig('./servo_data/rmse_double.png', dpi=300, bbox_inches='tight')
    
    # plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 15
    #6つのグラフの配置
    fig = plt.figure(figsize = (10,10))
    ax1 = fig.add_subplot(3,2,1)
    ax2 = fig.add_subplot(3,2,2)
    ax3 = fig.add_subplot(3,2,3)
    ax4 = fig.add_subplot(3,2,4)
    ax5 = fig.add_subplot(3,2,5)
    ax6 = fig.add_subplot(3,2,6)
    
    #各ループごとの各ジョイント角プロット
    ax1.plot(time_series, base_joint_data, color = 'blue', label = 'Current Joint Velocity')
    ax2.plot(time_series, shoulder_joint_data, color = 'blue', label = 'Current Joint Velocity')
    ax3.plot(time_series, elbow_joint_data, color = 'blue', label = 'Current Joint Velocity')
    ax4.plot(time_series, wrist1_joint_data, color = 'blue', label = 'Current Joint Velocity')
    ax5.plot(time_series, wrist2_joint_data, color = 'blue', label = 'Current Joint Velocity')
    ax6.plot(time_series, wrist3_joint_data, color = 'blue', label = 'Current Joint Velocity')
    
    # #目標角度をプロットscrewdriver
    ax1.axhline(y = 0.0, color = 'black')
    ax2.axhline(y = 0.0, color = 'black')
    ax3.axhline(y = 0.0, color = 'black')
    ax4.axhline(y = 0.0, color = 'black')
    ax5.axhline(y = 0.0, color = 'black')
    ax6.axhline(y = 0.0, color = 'black')
    
    #x軸のラベル
    ax1.set_xlabel('Time[s]', fontsize=18)
    ax2.set_xlabel('Time[s]', fontsize=18)
    ax3.set_xlabel('Time[s]', fontsize=18)
    ax4.set_xlabel('Time[s]', fontsize=18)
    ax5.set_xlabel('Time[s]', fontsize=18)
    ax6.set_xlabel('Time[s]', fontsize=18)
    
    #y軸のラベル
    ax1.set_ylabel('Base Joint Angular \n Velocity[rad/s]', fontsize=18)
    ax2.set_ylabel('Shoulder Joint Angular \n Velocity[rad/s]', fontsize=18)
    ax3.set_ylabel('Elbow Joint Angular \n Velocity[rad/s]', fontsize=18)
    ax4.set_ylabel('Wrist1 Joint Angular \n Velocity[rad/s]', fontsize=18)
    ax5.set_ylabel('Wrist2 Joint Angular \n Velocity[rad/s]', fontsize=18)
    ax6.set_ylabel('Wrist3 Joint Angular \n Velocity[rad/s]', fontsize=18)
    
    #凡例表示
    ax1.legend(ncol = 2, loc = 'lower right')
    ax2.legend(ncol = 2, loc = 'lower right')
    ax3.legend(ncol = 2, loc = 'lower right')
    ax4.legend(ncol = 2, loc = 'lower right')
    ax5.legend(ncol = 2, loc = 'lower right')
    ax6.legend(ncol = 2, loc = 'lower right')
    
    #重なり解消
    plt.tight_layout()
    
    ax1.grid()
    ax2.grid()
    ax3.grid()
    ax4.grid()
    ax5.grid()
    ax6.grid()
    
    #縦軸横軸の最大値、最小値、目盛り自動設定のグラフ保存
    # fig.savefig('./servo_data/joint_vel_values.png', dpi=300, bbox_inches='tight')
    fig.savefig('./servo_data/ex2/joint_vel_values.png')
    
    # base_axis_array = np.arange(-0.04, 0.04, 0.01)
    # shoulder_axis_array = np.arange(-0.04, 0.04, 0.01)
    # elbow_axis_array = np.arange(-0.04, 0.04, 0.01)
    # wrist1_axis_array = np.arange(-0.04, 0.04, 0.01)
    # wrist2_axis_array = np.arange(-0.04, 0.04, 0.01)
    # wrist3_axis_array = np.arange(-0.04, 0.04, 0.01) 
    
    # #グラフの目盛りのスケールを合わせる
    # ax1.set_yticks(base_axis_array)
    # ax2.set_yticks(shoulder_axis_array)
    # ax3.set_yticks(elbow_axis_array)
    # ax4.set_yticks(wrist1_axis_array)
    # ax5.set_yticks(wrist2_axis_array)
    # ax6.set_yticks(wrist3_axis_array)
    
    # y-axisの最大値と最小値を計算
    y_min = min(min(base_joint_data), min(shoulder_joint_data), min(elbow_joint_data), min(wrist1_joint_data), min(wrist2_joint_data), min(wrist3_joint_data))
    y_max = max(max(base_joint_data), max(shoulder_joint_data), max(elbow_joint_data), max(wrist1_joint_data), max(wrist2_joint_data), max(wrist3_joint_data))

    # 各サブプロットのy-axisのスケールを一致させる
    ax1.set_ylim(y_min, y_max)
    ax2.set_ylim(y_min, y_max)
    ax3.set_ylim(y_min, y_max)
    ax4.set_ylim(y_min, y_max)
    ax5.set_ylim(y_min, y_max)
    ax6.set_ylim(y_min, y_max)
    
    #スケールを合わせたグラフを保存
    fig.savefig('./servo_data/ex2/joint_values_double_scaled.png')
    # fig.savefig('./servo_data/joint_values_double_scaled.png', dpi=300, bbox_inches='tight')
    
    fig_dist = plt.figure()
    # plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 15
    plt.xlabel('Time[s]', fontsize=18)
    plt.ylabel('Three-Dimensional Distance[mm]', fontsize=18)
    plt.plot(time_series, dist_data, 'b-')
    plt.tight_layout()
    plt.grid()
    fig_dist.savefig('./servo_data/ex2/3dposedist.png')
    # fig_dist.savefig('./servo_data/3dposedist.png', dpi=300, bbox_inches='tight')

    sys.exit(0)

def main(msg):
    global pinv_int_mat_double
    global I_dsr_vec
    
    global rmseth
    
    global base_joint_data
    global shoulder_joint_data
    global elbow_joint_data
    global wrist1_joint_data
    global wrist2_joint_data
    global wrist3_joint_data
    
    global joint_vel_values_data
    
    global time_series
    global rmse_data
    global pose_data
    global dist_data
    
    # rmse = 100
    
    with open('./dsrth_result/desired_pose.csv', 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            desired_pose = [float(x) for x in row]
        
    image_raw = msg
    bridge = CvBridge()
    bgr = bridge.imgmsg_to_cv2(image_raw, 'bgr8') 
    # bgr = bridge.imgmsg_to_cv2(image_raw, 'mono8') ###UI camera
    gry = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gry2 = np.array(gry, dtype = 'float64')
    I_vec = gry2.reshape(-1,1)
    
    #画像偏差計算
    dI = I_dsr_vec - I_vec ###目標-現在
    dI2 = dI**2
    Isum = np.sum(dI2)
    rmse = math.sqrt(Isum / nop)
    print('rmse = %f' % rmse)
    
    rospy.wait_for_service('getpose')
    getpose_client = rospy.ServiceProxy('getpose', GettfPose)
    respose = getpose_client()
    # print(respose)
    current_pose_x = respose.trans[0] ###tfも単位[m]で記録されてる
    current_pose_y = respose.trans[1]
    current_pose_z = respose.trans[2]
    
    if rmse < rmseth:
    # if abs(desired_pose[0] - current_pose_x) < 0.0002 and abs(desired_pose[1] - current_pose_y) < 0.0002:
        print('stop')
        vel_input.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        vel_pub.publish(vel_input)
        print('stop command sent')
        
        filename = './servo_data/ex2/loop_rmse_data.csv'
        filename2 = './servo_data/ex2/joint_vel_data.csv'
        filename3 = './servo_data/ex2/pose_data.csv'
        with open (filename, 'w') as f, open(filename2, 'w')as f2, open(filename3, 'w')as f3: 
            writer = csv.writer(f)
            writer.writerow(rmse_data)
            writer2 = csv.writer(f2)
            writer2.writerow(joint_vel_values_data)
            writer3 = csv.writer(f3)
            writer3.writerow(pose_data)

        #generate graph
        fig_rmse = plt.figure()
        # plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'stix'
        plt.rcParams['font.size'] = 15
        plt.xlabel('Time[s]')
        plt.ylabel('RMSE')
        plt.plot(time_series, rmse_data, 'b-')
        plt.grid()
        fig_rmse.savefig('./servo_data/ex2/rmse_double.png')
        # fig_rmse.savefig('./servo_data/rmse_double.png', dpi=300, bbox_inches='tight')
        
        # plt.rcParams['font.family'] = 'Times New Roman'
        plt.rcParams['mathtext.fontset'] = 'stix'
        plt.rcParams['font.size'] = 15
        #6つのグラフの配置
        fig = plt.figure(figsize = (10,10))
        ax1 = fig.add_subplot(3,2,1)
        ax2 = fig.add_subplot(3,2,2)
        ax3 = fig.add_subplot(3,2,3)
        ax4 = fig.add_subplot(3,2,4)
        ax5 = fig.add_subplot(3,2,5)
        ax6 = fig.add_subplot(3,2,6)
        
        #各ループごとの各ジョイント角プロット
        ax1.plot(time_series, base_joint_data, color = 'blue', label = 'Current Joint Velocity')
        ax2.plot(time_series, shoulder_joint_data, color = 'blue', label = 'Current Joint Velocity')
        ax3.plot(time_series, elbow_joint_data, color = 'blue', label = 'Current Joint Velocity')
        ax4.plot(time_series, wrist1_joint_data, color = 'blue', label = 'Current Joint Velocity')
        ax5.plot(time_series, wrist2_joint_data, color = 'blue', label = 'Current Joint Velocity')
        ax6.plot(time_series, wrist3_joint_data, color = 'blue', label = 'Current Joint Velocity')
        
        # #目標角度をプロットscrewdriver
        ax1.axhline(y = 0.0, color = 'black')
        ax2.axhline(y = 0.0, color = 'black')
        ax3.axhline(y = 0.0, color = 'black')
        ax4.axhline(y = 0.0, color = 'black')
        ax5.axhline(y = 0.0, color = 'black')
        ax6.axhline(y = 0.0, color = 'black')
        
        #x軸のラベル
        ax1.set_xlabel('Time[s]', fontsize=18)
        ax2.set_xlabel('Time[s]', fontsize=18)
        ax3.set_xlabel('Time[s]', fontsize=18)
        ax4.set_xlabel('Time[s]', fontsize=18)
        ax5.set_xlabel('Time[s]', fontsize=18)
        ax6.set_xlabel('Time[s]', fontsize=18)
        
        #y軸のラベル
        ax1.set_ylabel('Base Joint Angular \n Velocity[rad/s]', fontsize=18)
        ax2.set_ylabel('Shoulder Joint Angular \n Velocity[rad/s]', fontsize=18)
        ax3.set_ylabel('Elbow Joint Angular \n Velocity[rad/s]', fontsize=18)
        ax4.set_ylabel('Wrist1 Joint Angular \n Velocity[rad/s]', fontsize=18)
        ax5.set_ylabel('Wrist2 Joint Angular \n Velocity[rad/s]', fontsize=18)
        ax6.set_ylabel('Wrist3 Joint Angular \n Velocity[rad/s]', fontsize=18)
        
        #凡例表示
        ax1.legend(ncol = 2, loc = 'lower right')
        ax2.legend(ncol = 2, loc = 'lower right')
        ax3.legend(ncol = 2, loc = 'lower right')
        ax4.legend(ncol = 2, loc = 'lower right')
        ax5.legend(ncol = 2, loc = 'lower right')
        ax6.legend(ncol = 2, loc = 'lower right')
        
        #重なり解消
        plt.tight_layout()
        
        ax1.grid()
        ax2.grid()
        ax3.grid()
        ax4.grid()
        ax5.grid()
        ax6.grid()
        
        #縦軸横軸の最大値、最小値、目盛り自動設定のグラフ保存
        fig.savefig('./servo_data/ex2/joint_vel_values.png')
        # fig.savefig('./servo_data/joint_vel_values.png', dpi=300, bbox_inches='tight')
        
        # y-axisの最大値と最小値を計算
        y_min = min(min(base_joint_data), min(shoulder_joint_data), min(elbow_joint_data), min(wrist1_joint_data), min(wrist2_joint_data), min(wrist3_joint_data))
        y_max = max(max(base_joint_data), max(shoulder_joint_data), max(elbow_joint_data), max(wrist1_joint_data), max(wrist2_joint_data), max(wrist3_joint_data))

        # 各サブプロットのy-axisのスケールを一致させる
        ax1.set_ylim(y_min, y_max)
        ax2.set_ylim(y_min, y_max)
        ax3.set_ylim(y_min, y_max)
        ax4.set_ylim(y_min, y_max)
        ax5.set_ylim(y_min, y_max)
        ax6.set_ylim(y_min, y_max)

        # 保存
        fig.savefig('./servo_data/ex2/joint_values_double_scaled.png')
        # fig.savefig('./servo_data/joint_values_double_scaled.png', dpi=300, bbox_inches='tight')
        
        fig_dist = plt.figure()
        # plt.rcParams['font.family'] = 'Times New Roman' ### times new roman not found
        plt.rcParams['mathtext.fontset'] = 'stix'
        plt.rcParams['font.size'] = 15
        plt.xlabel('Time[s]', fontsize=18)
        plt.ylabel('Three-Dimensional Distance[mm]', fontsize=18)
        plt.plot(time_series, dist_data, 'b-')
        plt.tight_layout()
        plt.grid()
        fig_dist.savefig('./servo_data/ex2/3dposedist.png')
        # fig_dist.savefig('./servo_data/3dposedist.png', dpi=300, bbox_inches='tight')

        rospy.signal_shutdown('finish')
    
    else:
        vel_input.data = lmbd*(np.dot(pinv_int_mat_double, dI)) 
        vel_pub.publish(vel_input) #プログラミングROS P109では速度の値だけf分岐させてpublish部分はif分岐の外においてた
        #serviceで関節角速度取得
        rospy.wait_for_service('getvel')
        getvel_client = rospy.ServiceProxy('getvel', GetCurrentJointVel)
        resvel = getvel_client()
        # print(resvel)
        
        current_base_vel = resvel.current_joint_vel[0]
        current_shoulder_vel = resvel.current_joint_vel[1]
        current_elbow_vel = resvel.current_joint_vel[2]
        current_wrist1_vel = resvel.current_joint_vel[3]
        current_wrist2_vel = resvel.current_joint_vel[4]
        current_wrist3_vel = resvel.current_joint_vel[5]
        
        ###pose が単位mで出力される
        ### 距離がしきい値以上で速度計算される場合に距離算出+append
        # dist = 1000*math.sqrt((desired_pose[0] - current_pose_x)**2 + (desired_pose[1] - current_pose_y)**2 + (desired_pose[2] - current_pose_z)**2)
        dist = 1000*math.sqrt((desired_pose[0] - current_pose_x)**2 + (desired_pose[1] - current_pose_y)**2) ###x,y軸方向だけの距離を出す
        
        current_time = time.time() - start_time
        time_series.append(current_time)
        rmse_data.append(rmse)
        base_joint_data.append(current_base_vel)
        shoulder_joint_data.append(current_shoulder_vel)
        elbow_joint_data.append(current_elbow_vel)
        wrist1_joint_data.append(current_wrist1_vel)
        wrist2_joint_data.append(current_wrist2_vel)
        wrist3_joint_data.append(current_wrist3_vel)
        joint_vel_values_data.append([current_base_vel, current_shoulder_vel, current_elbow_vel, current_wrist1_vel, current_wrist2_vel, current_wrist3_vel])
        pose_data.append([current_pose_x, current_pose_y, current_pose_z])
        dist_data.append(dist)
        
        return rmse, time_series, rmse_data, base_joint_data, shoulder_joint_data, elbow_joint_data, wrist1_joint_data, wrist2_joint_data, wrist3_joint_data, joint_vel_values_data, pose_data, dist_data
        

    
if __name__ == '__main__':
    try:
        rospy.init_node('servo_node')
        rospy.loginfo('servo node started')
        switch_to_joint_group_vel()
        atexit.register(switch_to_scaled_pos)
        readpickle()
        gen_dsrimvec()
        vel_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        vel_input = Float64MultiArray()
        vel_input.layout.data_offset = 1
        start_time = time.time()
        rospy.Subscriber('/camera/color/image_raw', Image, main)
        # rospy.Subscriber('/camera/image_raw', Image, main) #for UI camera 
        signal.signal(signal.SIGINT, signal_handler)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        switch_to_scaled_pos()
        rospy.signal_shutdown('finish')
    
    
#     Exception RuntimeError: 'main thread is not in main loop' in <bound method StringVar.__del__ of <Tkinter.StringVar instance at 0x7f63d3100460>> ignored
# Error in atexit._run_exitfuncs:
# Traceback (most recent call last):
#   File "/usr/lib/python2.7/atexit.py", line 24, in _run_exitfuncs
#     func(*targs, **kargs)
#   File "/home/kappa/.local/lib/python2.7/site-packages/matplotlib/_pylab_helpers.py", line 78, in destroy_all
#     manager.destroy()
#   File "/home/kappa/.local/lib/python2.7/site-packages/matplotlib/backends/_backend_tk.py", line 560, in destroy
#     self.window.destroy()
#   File "/usr/lib/python2.7/lib-tk/Tkinter.py", line 1872, in destroy
#     for c in self.children.values(): c.destroy()
#   File "/home/kappa/.local/lib/python2.7/site-packages/matplotlib/backends/_backend_tk.py", line 658, in destroy
#     Tk.Frame.destroy(self, *args)
#   File "/usr/lib/python2.7/lib-tk/Tkinter.py", line 2109, in destroy
#     for c in self.children.values(): c.destroy()
#   File "/usr/lib/python2.7/lib-tk/Tkinter.py", line 2110, in destroy
#     self.tk.call('destroy', self._w)
# RuntimeError: main thread is not in main loop
