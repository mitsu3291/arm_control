# -*- coding: utf-8 -*-
 
import numpy as np
import matplotlib.pyplot as plt
from math import *
import matplotlib.animation as animation

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

# 直線の軌道を生成 直線 端点の2つの座標を与える. 
def make_line_trajectory(start, end, req_t):
    cur_t = 0
    x_refs = []
    y_refs = []
    while cur_t < req_t:
        s = cur_t/req_t
        x_ref = start[0]*(1-s) + end[0]*s
        y_ref = start[1]*(1-s) + end[1]*s
        x_refs.append(x_ref)
        y_refs.append(y_ref)
        cur_t += 1
    
    return x_refs, y_refs

# 円の軌道を生成 始点と角度を与える
def calc_circle_trajectory(start, theta, req_t):
    cur_t = 0
    x_refs = []
    y_refs = []
    x1 = start[0]
    y1 = start[1]
    x_refs.append(x1)
    y_refs.append(y1)
    while cur_t < req_t:
        # 回転行列を計算
        r_mat = np.array([[cos(theta/req_t),-sin(theta/req_t)],
                          [sin(theta/req_t), cos(theta/req_t)]])
        
        x = np.array([[x1],
                      [y1]])
        
        x_new = np.dot(r_mat, x)
        x1 = x_new[0][0]
        y1 = x_new[1][0]
        x_refs.append(x1)
        y_refs.append(y1)
    
    return x_refs, y_refs

if __name__ == "__main__":
    # リンクの長さ
    l1 = 83
    l2 = 93.5
    l3 = 120

    # 描画用
    ims = []
    fig = plt.figure()
    flag_legend = True

    # params
    req_t = 200
    phi1 = 0.8
    phi2 = -0.1
    phi3 = 0.7

    cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,pi/2 - phi1,phi2,phi3)
    start = [cur_hand_pos[0][0], cur_hand_pos[1][0]]
    end = [cur_hand_pos[0][0], cur_hand_pos[1][0] - 100]
    x_refs, y_refs = make_line_trajectory(start, end, req_t) # 直線軌道を生成

    f = open('analytics/ik/last_ik.txt', 'r')
    datalist = f.readlines()
    f.close()

    pos_list = []
    for data in datalist:
        pos_list.append(data.split())

    for pos in pos_list:
        im = plt.plot([float(pos[0]),float(pos[1]),float(pos[2]),float(pos[3])],[float(pos[4]),float(pos[5]),float(pos[6]),float(pos[7])],'b-o',label="robot arm",color="m")
        ims.append(im)

        if flag_legend: # 一回のみ凡例を描画
            plt.plot(x_refs,y_refs,label="target trajectory")
            plt.xlim(-50,250)
            plt.ylim(-30,300)
            plt.axes().set_aspect('equal')
            plt.legend(loc='upper left')
            plt.title('Analytical Solution',loc='center')
            flag_legend = False

    ani = animation.ArtistAnimation(fig, ims, interval=1000)
    ani.save('analytics/ik/ik_real_ik.mp4', writer='ffmpeg',fps=12.5, dpi=300)
    fig.show()
    print("done")