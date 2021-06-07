#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import *
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np 
import matplotlib.animation as animation
import csv

# 2nd joint
def callback_joint2(data):
    global angle_2
    angle_2 = data.current_pos
    #rospy.loginfo("2nd angle : %f", angle_2)

# 3rd joint
def callback_joint3(data):
    global angle_3
    angle_3 = data.current_pos
    #rospy.loginfo("3nd angle : %f", angle_3)

# 4th joint
def callback_joint4(data):
    global angle_4
    angle_4 = data.current_pos
    #rospy.loginfo("4nd angle : %f", angle_4)

# 追従させる軌道 直線 端点の2つの座標を与える.  binは時間の分割数. 各時刻における所望の座標のリストを返す(indexは分割数個)
def calc_line_coodinate_bytime(start, end, req_t):
    cur_t = 0
    x_refs = []
    y_refs = []
    while(cur_t < req_t):
        r = cur_t*(req_t**(-1))
        s = 6*(r)**5 - 15*(r)**4 + 10*(r)**3
        x_ref = start[0]*(1-s) + end[0]*s
        y_ref = start[1]*(1-s) + end[1]*s
        x_refs.append(x_ref)
        y_refs.append(y_ref)
        cur_t += 1
    
    return x_refs, y_refs

def make_orbit(start, middle, end, add_mid, add_end):
    cur_t = 0
    x_refs = []
    y_refs = []
    while(cur_t < add_mid):
        #s = 6*(cur_t/req_t)**5 - 15*(cur_t/req_t)**4 + 10*(cur_t/req_t)**3
        s = cur_t*(add_mid**(-1))
        x_ref = start[0]*(1-s) + middle[0]*s
        y_ref = start[1]*(1-s) + middle[1]*s
        x_refs.append(x_ref)
        y_refs.append(y_ref)
        cur_t += 1

    cur_t = 0
    while(cur_t < add_end):
        s = cur_t*(add_end**(-1))
        x_ref = middle[0]*(1-s) + end[0]*s
        y_ref = middle[1]*(1-s) + end[1]*s
        #print(y_ref)
        x_refs.append(x_ref)
        y_refs.append(y_ref)
        cur_t += 1
    
    return x_refs, y_refs

# 順運動学で第一関節の位置を計算
def calc_cur_first_pos(l1,phi1):
    x = l1*cos(phi1)
    y = l1*sin(phi1)
    cur_first_pos = [x,y]

    return cur_first_pos

# 順運動学で第二関節の位置を計算
def calc_cur_second_pos(l1,l2,phi1,phi2):
    x = l1*cos(phi1) + l2*cos(phi1+phi2)
    y = l1*sin(phi1) + l2*sin(phi1+phi2)
    cur_second_pos = [x,y]

    return cur_second_pos

# 順運動学で現在の関節角度から手先位置を計算
def calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3):
    x = l1*cos(phi1) + l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3)
    y = l1*sin(phi1) + l2*sin(phi1+phi2) + l3*sin(phi1+phi2+phi3)
    alpha = phi1+phi2+phi3

    cur_hand_pos = np.array([[x],
                             [y],
                             [alpha]])

    return cur_hand_pos

def write_csv(joint_pos):
    f = open('last_ik.txt', 'w')
    for pos in joint_pos:
        f.writelines(pos)

    """
    with open('joint_pos.csv', 'a', newline="") as f:
        writer = csv.writer(f)
        for pos in joint_pos:
            writer.writerow(pos)
    """

if __name__ == '__main__':
    # initialization
    rospy.init_node('plotter')
    rospy.loginfo('started')

    # 角度の初期化
    # 冗長ヤコビ
    """
    phi1 = 1.8
    phi2 = 0.5
    phi3 = -0.5
    """
    phi1 = 0.056246
    phi2 = -0.751651
    phi3 = -0.025566

    # 続いたやつ
    phi1 = -0.542007
    phi2 = -0.659612
    phi3 = 0.388608

    """
    phi1 = 0.209644
    phi2 = 0.189191
    phi3 = -0.403948

    phi1 = 0.209644
    phi2 = 0.393722
    phi3 = -0.403948

    phi1 = -0.756764
    phi2 = -0.501100
    phi3 = -0.209644
    """

    angle_2 = 0
    angle_3 = 0
    angle_4 = 0

    # リンクの長さ
    l1 = 83
    l2 = 93.5
    l3 = 120

    # 処理にかかる時間
    req_t = 200

    # start end
    # 直線用
    start = calc_cur_hand_pos(l1,l2,l3,pi/2 - phi1, phi2, phi3)
    end = np.array([[start[0][0]+100],
                    [start[1][0]],
                    [0]]
                    )
    x_refs, y_refs = calc_line_coodinate_bytime(start, end, req_t)

    # 直線軌道用
    """
    cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
    start = [cur_hand_pos[0][0], cur_hand_pos[1][0]]
    middle = [cur_hand_pos[0][0] + 200, cur_hand_pos[1][0]]
    end = [cur_hand_pos[0][0] + 200, cur_hand_pos[1][0] - 120]
    add_mid = 200
    add_end = 120
    x_refs, y_refs = make_orbit(start, middle, end, add_mid, add_end)
    #print(len(y_refs))
    #print(y_refs)
    """

    # define listener
    joint2_sub = rospy.Subscriber('/tilt2_controller/state', JointState, callback_joint2)
    joint3_sub = rospy.Subscriber('/tilt3_controller/state', JointState, callback_joint3)
    joint4_sub = rospy.Subscriber('/tilt4_controller/state', JointState, callback_joint4)
    #rospy.spin()

    # 描画用
    sleepTime = 0.01
    fig = plt.figure()
    joint_pos = []

    while not rospy.is_shutdown():
        cur_first_pos = calc_cur_first_pos(l1, pi/2 - angle_2)
        cur_second_pos = calc_cur_second_pos(l1,l2,pi/2 - angle_2, angle_3)
        cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,pi/2 - angle_2, angle_3, angle_4)
        joint_pos.append([str(0)+'\t',str(cur_first_pos[0])+'\t',str(cur_second_pos[0])+'\t',str(cur_hand_pos[0][0])+'\t',str(0)+'\t',str(cur_first_pos[1])+'\t',str(cur_second_pos[1])+'\t',str(cur_hand_pos[1][0])+'\n'])
        plt.plot([0,cur_first_pos[0],cur_second_pos[0],cur_hand_pos[0][0]],[0,cur_first_pos[1],cur_second_pos[1],cur_hand_pos[1][0]],'b-o',label="robot arm",color="m")
        plt.plot(x_refs, y_refs, label="target trajectory")
        plt.legend()
        #plt.xlim(-170,170) #冗長ヤコビ
        #plt.ylim(-30,350) #冗長ヤコビ
        #plt.xlim(-30,350) # ヤコビ、解析
        #plt.ylim(-30,300) # ヤコビ、解析
        plt.axes().set_aspect('equal')
        plt.draw()
        plt.pause(sleepTime)
        plt.cla()

    write_csv(joint_pos)