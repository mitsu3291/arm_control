import numpy as np
import matplotlib.pyplot as plt
from math import *
import matplotlib.animation as animation
import csv
import time

# ヤコビ行列の疑似逆行列を求める
def calc_jacobian_inv(phi1, phi2, phi3, l1, l2, l3,error1, error2):
    # 冗長なので2*3行列
    jac = np.array([[-l1*sin(phi1) - l2*sin(phi1+phi2) - l3*sin(phi1+phi2+phi3),-l2*sin(phi1+phi2) - l3*sin(phi1+phi2+phi3),-l3*sin(phi1+phi2+phi3)],
                         [l1*cos(phi1) + l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3),l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3),l3*cos(phi1+phi2+phi3)]])

    eps = 1e-8
    eps_ = np.array([eps, eps])
    eps_diag = np.diag(eps_)
    try:
        jac_inv = jac.transpose() @ np.linalg.inv(jac @ jac.transpose() + eps_diag) #ヤコビ行列の逆行列を求める
        #jac_inv = jac.transpose() @ np.linalg.inv(jac @ jac.transpose())
    except:
        print("計算不可能です")

    return jac_inv

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
def calc_added_angle(cur_pos, x_refs, y_refs, index, yacobian_inv):
    goal_pos = np.array([[x_refs[index]],
                         [y_refs[index]]])
    cur_pos_ = np.delete(cur_pos, 2, 0) # 3行目の手先位置を削除
    dif_pos = goal_pos - cur_pos_ # 現在位置との差
    added_angles = np.dot(yacobian_inv, dif_pos) # 加えるべき関節角度の配列

    return added_angles

def write_csv(phi_list):
    with open('red_jac_phi_ver2_list.csv', 'a', newline="") as f:
        writer = csv.writer(f)
        for phis in phi_list:
            writer.writerow(phis)

if __name__ == "__main__":
    start_time = time.time()
    # params
    phi1 = 1.8
    phi2 = 0.5
    phi3 = -0.5

    # リンクの長さ
    l1 = 83
    l2 = 93.5
    l3 = 120

    # 初期位置
    cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
    #req_t = 300

    # 直線軌道用
    start = [cur_hand_pos[0][0], cur_hand_pos[1][0]]
    middle = [cur_hand_pos[0][0] + 215, cur_hand_pos[1][0] + 5]
    end = [cur_hand_pos[0][0] + 220, cur_hand_pos[1][0] - 150]
    add_mid = 220
    add_end = 150
    x_refs, y_refs = make_orbit(start, middle, end, add_mid, add_end)

    # 描画用
    ims = []
    fig = plt.figure()
    flag_legend = True

    phi_list = []

    # iteration
    for i in range(0,add_mid+add_end):
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

        fin_flag = False
        j = 0
        while (abs(error1) > 1 or abs(error2) > 1):
            if j == 10000:
                fin_flag = True
                break
            # ヤコビ逆行列を計算し、関節を追加する
            yacobian_inv = calc_jacobian_inv(l1,l2,l3,phi1,phi2,phi3, error1,error2)
            added_angles = calc_added_angle(cur_hand_pos, x_refs, y_refs, i, yacobian_inv)
            # チューニングゲイン
            K = 1
            phi1 += K*added_angles[0][0]
            phi2 += K*added_angles[1][0]
            phi3 += K*added_angles[2][0]
            cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
            error1 = x_refs[i] - cur_hand_pos[0][0]
            error2 = y_refs[i] - cur_hand_pos[1][0]
            j += 1

        #プロット用 遅くなるので10回に1回表示
        if i%1 == 0:
            im = plt.plot([0,cur_first_pos[0],cur_second_pos[0],cur_hand_pos[0][0]],[0,cur_first_pos[1],cur_second_pos[1],cur_hand_pos[1][0]],'b-o',label="robot arm",color="m")
            ims.append(im)
            #plt.plot([0,cur_first_pos[0],cur_second_pos[0],cur_hand_pos[0][0]],[0,cur_first_pos[1],cur_second_pos[1],cur_hand_pos[1][0]],'b-o',label="robot arm",color="m")
            #plt.plot(x_refs,y_refs,label="target trajectory")
            #plt.xlim(-170,250)
            #plt.ylim(-30,350)
            #plt.axes().set_aspect('equal')
            #plt.legend()
            #flag_legend = False
            #plt.draw()
            #plt.pause(sleepTime)
            #plt.cla()

        if flag_legend: # 一回のみ凡例を描画
            plt.plot(x_refs,y_refs,label="target trajectory")
            plt.xlim(-170,250)
            plt.ylim(-30,350)
            plt.axes().set_aspect('equal')
            plt.legend()
            flag_legend = False
        
        if fin_flag:
            break

    write_csv(phi_list)
    print(phi_list)
    elapsed_time = time.time() - start_time
    print ("elapsed_time:{0}".format(elapsed_time) + "[sec]")

    ani = animation.ArtistAnimation(fig, ims, interval=1)
    ani.save('sim_red_jac_able_singular.mp4', writer='ffmpeg',fps=10, dpi=300)
    fig.show()
    print("done")