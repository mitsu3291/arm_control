#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from math import *
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import numpy as np 
import matplotlib.animation as animation
import time
import matplotlib.pyplot as plt

# 追従させる軌道 直線 端点の2つの座標を与える.  binは時間の分割数. 各時刻における所望の座標のリストを返す(indexは分割数個)
def make_orbit(start, end, req_t):
    cur_t = 0
    x_refs = []
    y_refs = []
    while(cur_t < req_t):
        r = cur_t*(req_t**(-1))
        #s = 6*(r)**5 - 15*(r)**4 + 10*(r)**3
        s = r
        x_ref = start[0]*(1-s) + end[0]*s
        y_ref = start[1]*(1-s) + end[1]*s
        x_refs.append(x_ref)
        y_refs.append(y_ref)
        cur_t += 1
    
    return x_refs, y_refs

# ヤコビ行列の逆行列を求める
def calc_jacobian_inv(phi1, phi2, phi3, l1, l2, l3, error_x, error_y):
    # 冗長なので2*3行列
    jac = np.array([[-l1*sin(phi1) - l2*sin(phi1+phi2) - l3*sin(phi1+phi2+phi3),-l2*sin(phi1+phi2) - l3*sin(phi1+phi2+phi3),-l3*sin(phi1+phi2+phi3)],
                    [l1*cos(phi1) + l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3),l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3),l3*cos(phi1+phi2+phi3)]])

    eps = 1e-4
    eps_ = np.array([eps, eps])
    eps_diag = np.diag(eps_)
    try:
        jac_inv = np.dot(jac.transpose(),np.linalg.inv(np.dot(jac,jac.transpose()) + eps_diag)) #ヤコビ行列の逆行列を求める
        #jac_inv = jac.transpose() @ np.linalg.inv(jac @ jac.transpose())
    except:
        print("計算不可能です")

    return jac_inv

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

# 加える関節角度Φ1,Φ2,Φ3を求める cur_posは3×1行列 [x,y,alpha] (numpy_array)
def calc_added_angle(cur_pos, x_refs, y_refs, index, jacobian_inv):
    goal_pos = np.array([[x_refs[index]],
                         [y_refs[index]]])
    cur_pos_ = np.delete(cur_pos, 2, 0) # 3行目の手先位置を削除
    dif_pos = goal_pos - cur_pos_ # 現在位置との差
    #rospy.loginfo("dif_pos")
    #rospy.loginfo(dif_pos)
    added_angles = np.dot(jacobian_inv, dif_pos) # 加えるべき関節角度の配列

    return added_angles

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

def write_dif_pos(params):
    f = open('dif_pos.txt', 'w')
    for pos in params:
        f.writelines(pos)

def write_phi_list(params):
    f = open('phi_list.txt', 'w')
    for phis in params:
        f.writelines(phis)


if __name__ == "__main__":
    # initialization
    rospy.init_node('jac_compl_3link')
    rospy.loginfo("started")

    # 1秒あたりの送信回数
    rate = rospy.Rate(10)

    # define publisher
    joint1_pub = rospy.Publisher('/tilt1_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher('/tilt2_controller/command', Float64, queue_size=10)
    joint3_pub = rospy.Publisher('/tilt3_controller/command', Float64, queue_size=10)
    joint4_pub = rospy.Publisher('/tilt4_controller/command', Float64, queue_size=10)
    joint5_pub = rospy.Publisher('/tilt5_controller/command', Float64, queue_size=10)

    # define listener
    joint2_sub = rospy.Subscriber('/tilt2_controller/state', JointState, callback_joint2)
    joint3_sub = rospy.Subscriber('/tilt3_controller/state', JointState, callback_joint3)
    joint4_sub = rospy.Subscriber('/tilt4_controller/state', JointState, callback_joint4)

    # init
    angle_2 = 0.8
    angle_3 = -0.1
    angle_4 = 0.7

    # adjust 右のやつ
    phi1 = 0.056246
    phi2 = -0.751651
    phi3 = -0.025566

    # adjust
    """
    phi1 = 0.363042
    phi2 = 0.705631
    phi3 = 0.199418

    phi1 = 0.158511
    phi2 = 0.393722
    phi3 = 0.199418

    phi1 = -0.542007
    phi2 = -0.659612
    phi3 = 0.388608

    phi1 = 0.209644
    phi2 = 0.393722
    phi3 = -0.403948
    """

    # リンクの長さ
    l1 = 83
    l2 = 93.5
    l3 = 120

    #sleepTime = 0.01
    # 値を渡す
    """
    phi1 = 1.536922657419879
    phi2 = -0.7235753483252765
    phi3 = -0.013347309094605464
    """

    # 初期位置
    cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,pi/2 - phi1,phi2,phi3)

    # 直線軌道用
    req_t = 100 #100
    start = [cur_hand_pos[0][0], cur_hand_pos[1][0]]
    end = [cur_hand_pos[0][0], cur_hand_pos[1][0]-100]
    x_refs, y_refs = make_orbit(start,end,req_t)

    #joint2_pub.publish(pi/2 - 1.536922657419879)
    #joint3_pub.publish(-0.7235753483252765)
    #joint4_pub.publish(-0.013347309094605464)
    time.sleep(3)
    tmp1 = 0
    tmp2 = 0
    tmp3 = 0

    while not rospy.is_shutdown():
        for i in range(1,req_t):
            # 違う変数に値入れておかないとfor文の中で更新されてしまう
            phi1 = pi/2 - angle_2
            phi2 = angle_3
            phi3 = angle_4
            rate.sleep()
            #rospy.loginfo(i)
            #rospy.loginfo([phi1,phi2,phi3])
            phi_list = []
            phi_list.append([str(phi1)+'\t',str(phi2)+'\t',str(phi3)+'\n'])
            # 手先の現在位置を計算
            cur_first_pos = calc_cur_first_pos(l1, phi1)
            cur_second_pos = calc_cur_second_pos(l1,l2,phi1, phi2)
            cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)

            # 差を出力
            dif_poses = []
            cur_pos_ = np.delete(cur_hand_pos, 2, 0)
            goal_pos = np.array([[x_refs[i-1]],
                                 [y_refs[i-1]]])
            dif_pos = goal_pos - cur_pos_
            rospy.loginfo([dif_pos[0][0],dif_pos[1][0]])
            rospy.loginfo([tmp1 - phi1, tmp2 - phi2, tmp3 - phi3])
            rospy.loginfo([tmp1, tmp2, tmp3])
            dif_poses.append([str(dif_pos[0][0])+'\t',str(dif_pos[1][0])+'\n'])

            #rospy.loginfo(cur_hand_pos)
            # 目標値との誤差  適当に初期化
            error_x = x_refs[i-1] - cur_hand_pos[0][0]
            error_y = y_refs[i-1] - cur_hand_pos[1][0]

            j = 0
            while (abs(error_x) > 1 or abs(error_y) > 1):
                j += 1
                # ヤコビ逆行列を計算し、関節を追加する
                jacobian_inv = calc_jacobian_inv(l1,l2,l3,phi1,phi2,phi3, error_x, error_y)
                added_angles = calc_added_angle(cur_hand_pos, x_refs, y_refs, i, jacobian_inv)
                # チューニングゲイン
                K = 1
                phi1 += K*((added_angles[0][0])%(2*pi))
                phi2 += K*((added_angles[1][0])%(2*pi))
                phi3 += K*((added_angles[2][0])%(2*pi))
                cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
                error_x = x_refs[i-1] - cur_hand_pos[0][0]
                error_y = y_refs[i-1] - cur_hand_pos[1][0]
                if j == 200000:
                    print("too heavy")
                    exit()
            #rospy.loginfo(jacobian_inv)
            #rospy.loginfo([phi1,phi2,phi3])

            tmp1 = phi1%(2*pi)
            tmp2 = phi2%(2*pi)
            tmp3 = phi3%(2*pi)
            if tmp1 > pi:
                tmp1 -= 2*pi
            if tmp2 > pi:
                tmp2 -= 2*pi
            if tmp3 > pi:
                tmp3 -= 2*pi

            # 動作できないところだったら止める
            if tmp1 < 0.2 or tmp2 < -3*pi/4 or tmp2 > 3*pi/4 or tmp3 < -pi/2:
                print("cannot move")
                print("phi1")
                print(tmp1)
                print("phi2")
                print(tmp2)
                print("phi3")
                print(tmp3)
                exit()
            joint2_pub.publish(pi/2 - tmp1)
            joint3_pub.publish(tmp2)
            joint4_pub.publish(tmp3)
            time.sleep(2) #移動までの時間

        rospy.spin()
        exit()