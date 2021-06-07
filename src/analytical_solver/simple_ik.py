#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import *
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import numpy as np 

# 逆運動学を解いて各関節角度を出力
def calc_ik(goal_pos, l1, l2, l3):
    x_ref = goal_pos[0][0]
    y_ref = goal_pos[1][0]
    alpha = goal_pos[2][0]
    
    A = x_ref - (l3*cos(alpha))
    B = y_ref - (l3*sin(alpha))
    phi1 = atan2(B,A) + acos((A**2+B**2+l1**2-l2**2)/(2*l1*sqrt(A**2+B**2)))
    phi2 = -phi1 + atan2((B-l1*sin(phi1)),(A-l1*cos(phi1)))
    phi3 = alpha - phi1 - phi2
                    
    return phi1,phi2,phi3

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

# 直線軌道での所望の座標を出力 start, endはnumpy_array
def make_next_pos(start, end, cur_t, req_t):
    s = 6*(cur_t/req_t)**5 - 15*(cur_t/req_t)**4 + 10*(cur_t/req_t)**3
    goal_pos = start*(1-s) + end*s
    
    return goal_pos 

if __name__ == "__main__":
    # initialization
    rospy.init_node('analytical_solver')
    rospy.loginfo("started")

    # 1秒あたりの送信回数
    rate = rospy.Rate(10)

    # define publisher
    joint1_pub = rospy.Publisher('/tilt1_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher('/tilt2_controller/command', Float64, queue_size=10)
    joint3_pub = rospy.Publisher('/tilt3_controller/command', Float64, queue_size=10)
    joint4_pub = rospy.Publisher('/tilt4_controller/command', Float64, queue_size=10)
    joint5_pub = rospy.Publisher('/tilt5_controller/command', Float64, queue_size=10)

    phi1 = 1.8
    phi2 = -0.1
    phi3 = 0.7

    # リンクの長さ
    l1 = 83
    l2 = 93.5
    l3 = 120

    # 初期位置, 終了位置
    start = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
    end = np.array([[start[0][0]],
                    [start[1][0] - 100],
                    [0]])

    # iteration
    req_t = 100

    while not rospy.is_shutdown():
        for cur_t in range(1,req_t):
            goal_pos = make_next_pos(start, end, cur_t, req_t)
            phi1, phi2, phi3 = calc_ik(goal_pos, l1, l2, l3)
            joint2_pub.publish(pi/2 - phi1)
            joint3_pub.publish(phi2)
            joint4_pub.publish(phi3)

        rospy.spin()
        print("done")
        exit()