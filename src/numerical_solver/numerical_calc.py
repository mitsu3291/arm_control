import numpy as np
import matplotlib.pyplot as plt
from math import *
import matplotlib.animation as animation
import csv
import time

# ヤコビ行列の逆行列を求める
def calc_jacobian_inv(phi1, phi2, phi3, l1, l2, l3):
    jacobian = np.array([[-l1*sin(phi1) - l2*sin(phi1+phi2) - l3*sin(phi1+phi2+phi3),-l2*sin(phi1+phi2) - l3*sin(phi1+phi2+phi3),-l3*sin(phi1+phi2+phi3)],
                         [l1*cos(phi1) + l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3),l2*cos(phi1+phi2) + l3*cos(phi1+phi2+phi3),l3*cos(phi1+phi2+phi3)],
                         [1,1,1]])
    try:
        jacobian_inv = np.linalg.inv(jacobian) #ヤコビ行列の逆行列を求める
        print(np.linalg.det(jacobian))
    except np.linalg.LinAlgError as err:
        print("singular position")
        exit()

    return jacobian_inv

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
                         [y_refs[index]],
                         [1.8]]) #αは0とする
    dif_pos = goal_pos - cur_pos # 現在位置との差
    added_angles = np.dot(yacobian_inv, dif_pos) # 加えるべき関節角度の配列

    return added_angles

def write_csv(phi_list):
    with open('jac_phi_list.csv', 'a', newline="") as f:
        writer = csv.writer(f)
        for phis in phi_list:
            writer.writerow(phis)

if __name__ == "__main__":
    # initialization
    rospy.init_node('Numerical_solver')
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
    phi1 = 0.8
    phi2 = -0.1
    phi3 = 0.7

    angle_2 = 0
    angle_3 = 0
    angle_4 = 0

    # リンクの長さ
    l1 = 83
    l2 = 93.5
    l3 = 120

    # 初期位置
    cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,pi/2 - phi1,phi2,phi3)

    # 直線軌道
    req_t = 100
    start = [cur_hand_pos[0][0], cur_hand_pos[1][0]]
    end = [cur_hand_pos[0][0], cur_hand_pos[1][0]-100]
    x_refs, y_refs = make_line_trajectory(start,end,req_t)

    time.sleep(1)
    tmp1 = 0
    tmp2 = 0
    tmp3 = 0

    while not rospy.is_shutdown():
        for i in range(1,req_t):
            # 別の変数にいれて更新を防ぐ
            phi1 = pi/2 - angle_2
            phi2 = angle_3
            phi3 = angle_4
            rate.sleep()

            phi_list = []
            phi_list.append([str(phi1)+'\t',str(phi2)+'\t',str(phi3)+'\n'])
            # 手先の現在位置を計算
            cur_first_pos = calc_cur_first_pos(l1, phi1)
            cur_second_pos = calc_cur_second_pos(l1,l2,phi1, phi2)
            cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)

            # 目標値からの誤差
            error_x = x_refs[i] - cur_hand_pos[0][0]
            error_y = y_refs[i] - cur_hand_pos[1][0]

            while (abs(error_x) > 1 or abs(error_y) > 1):
                # ヤコビ逆行列を計算し、関節角度を追加する
                jacobian_inv = calc_jacobian_inv(l1,l2,l3,phi1,phi2,phi3)
                added_angles = calc_added_angle(cur_hand_pos, x_refs, y_refs, i, jacobian_inv)

                # チューニングゲイン
                K = 1
                phi1 += K*(added_angles[0][0])
                phi2 += K*(added_angles[1][0])
                phi3 += K*(added_angles[2][0])
                cur_hand_pos = calc_cur_hand_pos(l1,l2,l3,phi1,phi2,phi3)
                error_x = x_refs[i] - cur_hand_pos[0][0]
                error_y = y_refs[i] - cur_hand_pos[1][0]


            # 2π超えとるから修正
            tmp1 = phi1%(2*pi)
            tmp2 = phi2%(2*pi)
            tmp3 = phi3%(2*pi)
            if tmp1 > pi:
                tmp1 -= 2*pi
            if tmp2 > pi:
                tmp2 -= 2*pi
            if tmp3 > pi:
                tmp3 -= 2*pi

            # 可動域外だったら終了
            if tmp1 < 0.2 or tmp2 < -3*pi/4 or tmp2 > 3*pi/4 or tmp3 < -pi/2:
                print("cannot move")
                exit()

            joint2_pub.publish(pi/2 - tmp1)
            joint3_pub.publish(tmp2)
            joint4_pub.publish(tmp3)
            time.sleep(1) #移動までの時間

        rospy.spin()
        exit()