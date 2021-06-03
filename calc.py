import numpy as np
import matplotlib.pyplot as plt
from math import *
import matplotlib.animation as animation
import csv
import time

phi1 = 2.2 
phi2 = -0.40000000000000036
phi3 = -0.09999999999999964
"""
phi1 = 2.2 
phi2 = -0.4
phi3 = -0.1
"""

# ヤコビ行列の疑似逆行列を求める
def calc_jacobian_inv(phi1, phi2, phi3, l1, l2, l3):
    # 冗長なので2*3行列
    jac = np.array([[-l1*sin(phi1) - l2*sin(phi1+phi2) - l3*sin(phi1+phi2+phi3),-l2*sin(phi1+phi2) - l3*sin(phi1+phi2+phi3),-l3*sin(phi1+phi2+phi3)],
                         [l1*cos(phi1) + l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3),l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3),l3*cos(phi1+phi2+phi3)]])

    eps = 1e-4
    eps_ = np.array([eps, eps])
    eps_diag = np.diag(eps_)
    try:
        jac_inv = jac.transpose() @ np.linalg.inv(jac @ jac.transpose() + eps_diag) #ヤコビ行列の逆行列を求める
        #jac_inv = jac.transpose() @ np.linalg.inv(jac @ jac.transpose())
    except:
        print("計算不可能です")

    return jac_inv

# 順運動学で現在の関節角度から手先位置を計算
def calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3):
    x = l1*cos(phi1) + l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3)
    y = l1*sin(phi1) + l2*sin(phi1+phi2) + l3*sin(phi1+phi2+phi3)
    alpha = phi1+phi2+phi3

    cur_hand_pos = np.array([[x],
                             [y],
                             [alpha]])

    return cur_hand_pos

# 追従させる軌道 直線 端点の2つの座標（start, end）を与える. req_tは処理時間.
def make_orbit(start, end, req_t):
    cur_t = 0
    x_refs = []
    y_refs = []
    while(cur_t < req_t):
        #s = 6*(cur_t/req_t)**5 - 15*(cur_t/req_t)**4 + 10*(cur_t/req_t)**3
        if cur_t > 0:
            s = cur_t/req_t
            x_ref = start[0]*(1-s) + end[0]*s
            y_ref = start[1]*(1-s) + end[1]*s
            x_refs.append(x_ref)
            y_refs.append(y_ref)
        cur_t += 1
    
    return x_refs, y_refs

# 加える関節角度Φ1,Φ2,Φ3を求める
def calc_added_angle(cur_pos, x_refs, y_refs, index, yacobian_inv):
    goal_pos = np.array([[x_refs[index]],
                         [y_refs[index]]])
    cur_pos_ = np.delete(cur_pos, 2, 0) # 3行目の手先位置を削除
    dif_pos = goal_pos - cur_pos_ # 現在位置と目標位置との差
    added_angles = np.dot(yacobian_inv, dif_pos) # 加えるべき関節角度の配列

    return added_angles

if __name__ == "__main__":
    # params
    phi1 = 2.2
    phi2 = -0.4
    phi3 = -0.1

    phi1 = 2.2 
    phi2 = -0.4000001
    phi3 = -0.9999999

    # リンクの長さ
    l1 = 83
    l2 = 93.5
    l3 = 120

    # 初期位置
    cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
    print(cur_hand_pos)

    # 直線軌道用
    start = [cur_hand_pos[0][0], cur_hand_pos[1][0]]
    end = [cur_hand_pos[0][0] + 0.1, cur_hand_pos[1][0]]
    cur_t = 2
    x_refs, y_refs = make_orbit(start, end, cur_t)

    # iteration
    for i in range(0,cur_t-1):
        # 手先位置を計算
        cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)

        # 目標値との誤差  適当に初期化
        error1 = 2
        error2 = 2

        fin_flag = False
        j = 0
        
        while (abs(error1) > 1 or abs(error2) > 1):
            if j == 10000:
                fin_flag = True
                break
            # ヤコビ逆行列を計算し、関節を追加する
            yacobian_inv = calc_jacobian_inv(l1,l2,l3,phi1,phi2,phi3)
            added_angles = calc_added_angle(cur_hand_pos, x_refs, y_refs, i, yacobian_inv)
            # チューニングゲイン
            K = 1
            phi1 += K*(added_angles[0][0])
            phi2 += K*(added_angles[1][0])
            phi3 += K*(added_angles[2][0])
            cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
            error1 = x_refs[i] - cur_hand_pos[0][0]
            error2 = y_refs[i] - cur_hand_pos[1][0]
            j += 1

        #print([phi1,phi2,phi3])
        tmp1 = phi1 % (2*pi)
        tmp2 = phi2 % (2*pi)
        tmp3 = phi3 % (2*pi)
        if tmp1 > pi:
            tmp1 -= 2*pi
        if tmp2 > pi:
            tmp2 -= 2*pi
        if tmp3 > pi:
            tmp3 -= 2*pi
        print([tmp1,tmp2,tmp3])

        # test
        if i == 1:
            exit()
        
        if fin_flag:
            print("収束しません")
            break