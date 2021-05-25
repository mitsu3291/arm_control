import numpy as np
import matplotlib.pyplot as plt
from math import *
import matplotlib.animation as animation
import csv

def calc_jacobian_inv(phi1, phi2, phi3, l1, l2, l3):
    jacobian = np.array([[-l1*sin(phi1) - l2*sin(phi1+phi2) - l3*sin(phi1+phi2+phi3),-l2*sin(phi1+phi2) - l3*sin(phi1+phi2+phi3),-l3*sin(phi1+phi2+phi3)],
                         [l1*cos(phi1) + l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3),l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3),l3*cos(phi1+phi2+phi3)],
                         [1,1,1]])

    return jacobian

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

def write_csv(phi_list):
    with open('phi_list.csv', 'a', newline="") as f:
        writer = csv.writer(f)
        for phis in phi_list:
            writer.writerow(phis)

if __name__ == "__main__":
    # params
    phi1 = 0.8
    phi2 = -0.1
    phi3 = 0.7

    # リンクの長さ
    l1 = 83
    l2 = 93.5
    l3 = 120

    # 初期位置
    cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)

    # 円軌道用
    req_t = 300
    theta = -pi/4

    # 直線軌道用
    start = [cur_hand_pos[0][0], cur_hand_pos[1][0]]
    end = [cur_hand_pos[0][0], cur_hand_pos[1][0] - 130]
    x_refs, y_refs = calc_line_coodinate_bytime(start,end,req_t)

    # 描画用
    ims = []
    fig = plt.figure()
    flag_legend = True

    phi_list = []

    # iteration
    for i in range(0,req_t):
        tmp1 = phi1%(2*pi)
        tmp2 = phi2%(2*pi)
        tmp3 = phi3%(2*pi) 
        if tmp1 > pi:
            tmp1 -= 2*pi
        if tmp2 > pi:
            tmp2 -= 2*pi
        if tmp3 > pi:
            tmp3 -= 2*pi

        if tmp1 < -0.2:
            print("tmp1 out")
            print(i)
            print(tmp1)
            exit()
        if tmp2 < -3*pi/4 or tmp2 > 3*pi/4:
            print("tmp2 out")
            print(i)
            print(tmp2)
            exit()
        if tmp3 < -pi/2:
            print("tmp3 out")
            print(i)
            print(tmp3)
            exit()
        phi_list.append([tmp1,tmp2,tmp3])
        # 各関節の現在位置を計算
        cur_first_pos = calc_cur_first_pos(l1,phi1)
        cur_second_pos = calc_cur_second_pos(l1,l2,phi1,phi2)
        cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)

        # 目標値との誤差  適当に初期化
        error1 = 2
        error2 = 2
        sleepTime = 0.01

        W_N_ = np.diag(np.full(3, 0.001))
        W_E = np.diag(np.full(3, 1))
        e = np.array([[x_refs[i]-cur_hand_pos[0][0]],
                      [y_refs[i]-cur_hand_pos[1][0]]])
        for j in range(100):
            jac = calc_jacobian_inv(phi1, phi2, phi3, l1, l2, l3)
            val = e.transpose() @ W_E @ e / 2 
            W_N = val*3 + W_N_
            H = jac.transpose() @ W_E @ jac + W_N
            g = jac.transpose() @ W_E @ e
            del_q = np.linalg.inv(H) @ g
            phi1 += del_q[0][0]
            phi2 += del_q[1][0]
            phi3 += del_q[2][0]
            e = x_refs[i] - calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
            if e.transpose() @ W_E @ e / 2 < 0.1:
                break

        #プロット用 遅くなるので10回に1回表示
        if i%3 == 0:
            plt.plot([0,cur_first_pos[0],cur_second_pos[0],cur_hand_pos[0][0]],[0,cur_first_pos[1],cur_second_pos[1],cur_hand_pos[1][0]],'b-o',label="robot arm",color="m")
            plt.plot(x_refs,y_refs,label="target trajectory")
            plt.xlim(-100,300)
            plt.axes().set_aspect('equal')
            plt.legend()
            plt.draw()
            plt.pause(sleepTime)
            plt.cla()

    print("done")