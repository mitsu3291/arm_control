import numpy as np
import matplotlib.pyplot as plt
from math import *
import matplotlib.animation as animation
import time
import csv

# 逆運動学を解いて各関節角度を出力
def calc_ik(goal_pos, l1, l2, l3):
    x_ref = goal_pos[0][0]
    y_ref = goal_pos[1][0]
    alpha = goal_pos[2][0]
    
    A = x_ref - (l3*cos(alpha))
    B = y_ref - (l3*sin(alpha))
    #print((A**2+B**2+l1**2-l2**2)/(2*l1*sqrt(A**2+B**2)))
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

# 追従させる軌道 直線 端点の2つの座標を与える.  binは時間の分割数. 各時刻における所望の座標のリストを返す(indexは分割数個)
def make_orbit(start, end, req_t):
    cur_t = 0
    x_refs = []
    y_refs = []
    while(cur_t < req_t):
        s = 6*(cur_t/req_t)**5 - 15*(cur_t/req_t)**4 + 10*(cur_t/req_t)**3
        x_ref = start[0]*(1-s) + end[0]*s
        y_ref = start[1]*(1-s) + end[1]*s
        x_refs.append(x_ref)
        y_refs.append(y_ref)
        cur_t += 1
    
    return x_refs, y_refs

# 直線軌道での所望の座標を出力 start, endはnumpy_array
def make_goal_pos(start, end, cur_t, req_t):
    s = 6*(cur_t/req_t)**5 - 15*(cur_t/req_t)**4 + 10*(cur_t/req_t)**3
    goal_pos = start*(1-s) + end*s
    
    return goal_pos 

def write_csv(phi_list):
    with open('last_ik.csv', 'a', newline="") as f:
        writer = csv.writer(f)
        for phis in phi_list:
            writer.writerow(phis)

if __name__ == "__main__":
    start_time = time.time()

    phi1 = pi/2 - 0.056246
    phi2 = -0.751651
    phi3 = -0.025566

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
    x_refs, y_refs = make_orbit(start, end, req_t)
    sleepTime = 0.1
    phi_list = []
    for cur_t in range(req_t):
        goal_pos = make_goal_pos(start, end, cur_t, req_t)
        phi1, phi2, phi3 = calc_ik(goal_pos, l1, l2, l3)
        phi_list.append([phi1,phi2,phi3])
        cur_first_pos = calc_cur_first_pos(l1,phi1)
        cur_second_pos = calc_cur_second_pos(l1,l2,phi1,phi2)
        cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)

        plt.plot([0,cur_first_pos[0],cur_second_pos[0],cur_hand_pos[0][0]],[0,cur_first_pos[1],cur_second_pos[1],cur_hand_pos[1][0]],'b-o',label="robot arm",color="m")
        plt.plot(x_refs,y_refs,label="target trajectory")
        plt.xlim(-170,250)
        plt.ylim(-30,350)
        plt.axes().set_aspect('equal')
        plt.legend()
        flag_legend = False
        plt.draw()
        plt.pause(sleepTime)
        plt.cla()
    
    write_csv(phi_list)