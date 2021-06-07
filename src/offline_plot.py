import csv
import matplotlib.pyplot as plt 
from math import *

def get_phi_list():
    phi_list = []
    with open("last_phi_list.csv") as f:
        reader = csv.reader(f)
        for row in reader:
            phi_list.append(row)

    return phi_list

if __name__ == "__main__":
    phi_list = get_phi_list()
    phi1_list = []
    phi2_list = []
    phi3_list = []
    for phis in phi_list:
        phi1_list.append(float(phis[0]))
        phi2_list.append(float(phis[1]))
        phi3_list.append(float(phis[2]))

    indexes = [i+1 for i in range(len(phi3_list))]
    fig = plt.figure()
    plt.plot(indexes, phi3_list)
    plt.xlabel('step')
    plt.ylabel('input angle')
    plt.title('Joint 3',loc='center')
    plt.show()