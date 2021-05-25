import numpy as np
import matplotlib.pyplot as plt
from math import *
import matplotlib.animation as animation
import csv

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
    except:
        print("計算不可能です")

    return jac_inv

if __name__ == "__main__":
    # params
    phi1 = 0.8
    phi2 = -0.1
    phi3 = 0.7

    # リンクの長さ
    l1 = 83
    l2 = 93.5
    l3 = 120

    jac_inv = calc_jacobian_inv(phi1, phi2, phi3, l1, l2, l3)
    print(jac_inv)