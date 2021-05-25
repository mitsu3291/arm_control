#!/usr/bin/env python3
# -*- coding: utf-8 -*-

if __name__ == "__main__":
    f = open('myfile.txt', 'w')
    phi1 = 0.1
    phi2 = 0.2
    phi3 = 0.4
    for i in range(10):
        f.writelines([str(phi1)+'\t', str(phi2)+'\t', str(phi3)+'\n'])

    