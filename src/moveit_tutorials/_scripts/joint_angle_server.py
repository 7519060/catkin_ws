#!/usr/bin/env python
#coding: UTF-8

import rospy

from sensor_msgs.msg import JointState
# from std_msgs.msg import Float64MultiArray
# from moveit_tutorials.srv import GetCurrentJointVel, GetCurrentJointVelResponse
from moveit_tutorials.srv import GetCurrentJointPos, GetCurrentJointPosResponse

### /joint_statesを常にsubscribeし、client(6dmotion_and_capture_server.py)からのサービスコールがあったときにjoint states(pos = ジョイント角度)を返す
# vel = []
pos = []
    
#subsciberコールバック関数内では変数に取得したデータを代入
def get_joint_state(data):
    global pos
    pos = data.position
    return pos

#サービスコールのコールバックの中で変数(pos)を送信
def return_pos(res):
    global pos
    rospy.loginfo('sent joint angle response')
    # print('sent response')
    return GetCurrentJointPosResponse(pos)

def main():
	rospy.loginfo('ready to serve')
	rospy.Subscriber('/joint_states', JointState, get_joint_state)
	rospy.Service('getpos', GetCurrentJointPos, return_pos)
	rospy.spin()

if __name__ == '__main__':
	try:
		rospy.init_node('joint_pos_server')
		main()
	except rospy.ROSInterruptException:
		pass
