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

# 追従させる軌道 直線 端点の2つの座標を与える.  req_tは処理時間. 各時刻における所望の座標のリストを返す(indexは分割数個)
# 2つの直線がまじったようなものを考える
def make_orbit(start, middle, end, add_mid, add_end):
    cur_t = 0
    x_refs = []
    y_refs = []
    while(cur_t < add_mid):
        #s = 6*(cur_t/req_t)**5 - 15*(cur_t/req_t)**4 + 10*(cur_t/req_t)**3
        s = cur_t/add_mid
        x_ref = start[0]*(1-s) + middle[0]*s
        y_ref = start[1]*(1-s) + middle[1]*s
        x_refs.append(x_ref)
        y_refs.append(y_ref)
        cur_t += 1

    cur_t = 0
    while(cur_t < add_end):
        s = cur_t/add_mid
        x_ref = middle[0]*(1-s) + end[0]*s
        y_ref = middle[1]*(1-s) + end[1]*s
        x_refs.append(x_ref)
        y_refs.append(y_ref)
        cur_t += 1
    
    return x_refs, y_refs

# 追従させる軌道 直線 端点の2つの座標を与える.  req_tは処理時間. 各時刻における所望の座標のリストを返す(indexは分割数個)
def calc_line_coodinate_bytime(start, end, req_t):
    cur_t = 0
    x_refs = []
    y_refs = []
    while(cur_t < req_t):
        #s = 6*(cur_t/req_t)**5 - 15*(cur_t/req_t)**4 + 10*(cur_t/req_t)**3
        s = cur_t/req_t
        x_ref = start[0]*(1-s) + end[0]*s
        y_ref = start[1]*(1-s) + end[1]*s
        x_refs.append(x_ref)
        y_refs.append(y_ref)
        cur_t += 1
    
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

    # 解析、ヤコビ
    """
    req_t = 200
    phi1 = 0.8
    phi2 = -0.1
    phi3 = 0.7

    cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
    start = [cur_hand_pos[0][0], cur_hand_pos[1][0]]
    end = [cur_hand_pos[0][0], cur_hand_pos[1][0] - 140]
    x_refs, y_refs = calc_line_coodinate_bytime(start, end, req_t)
    """

    # 冗長ヤコビ
    phi1 = 1.8
    phi2 = 0.5
    phi3 = -0.5

    cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
    start = [cur_hand_pos[0][0]+20, cur_hand_pos[1][0]]
    middle = [cur_hand_pos[0][0] + 200, cur_hand_pos[1][0]] # +215 +5
    end = [cur_hand_pos[0][0] + 200, cur_hand_pos[1][0] - 150] # +220 -150
    add_mid = 200
    add_end = 150
    x_refs, y_refs = make_orbit(start, middle, end, add_mid, add_end)

    f = open('red_jac_pos.txt', 'r')
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
            plt.xlim(-170,170) # 冗長ヤコビ
            plt.ylim(-30,350) # 冗長ヤコビ
            #plt.xlim(-70,230) # 解析、ヤコビ
            #plt.ylim(-30,270) # 解析、ヤコビ
            plt.axes().set_aspect('equal')
            plt.legend(loc='upper left')
            plt.title('Levenberg-Marquardt Method',loc='center')
            flag_legend = False

    ani = animation.ArtistAnimation(fig, ims, interval=1000)
    ani.save('real_LM.mp4', writer='ffmpeg',fps=13.5, dpi=300)
    fig.show()
    print("done")