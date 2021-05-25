#!/usr/bin/env python3
# -*- coding: utf-8 -*-

if __name__ == "__main__":
    f = open('myfile.txt', 'r', encoding='UTF-8')
    datalist = f.readlines()
    for data in datalist:
        print(data.split())
    f.close()