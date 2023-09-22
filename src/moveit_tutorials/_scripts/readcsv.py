#!/usr/bin/env python
# coding: UTF-8

import csv
import numpy as np

with open('./dsrth_result/desired_pose.csv', 'r') as f:
    reader = csv.reader(f)
    for row in reader:
        arr = [float(x) * -1000 if i < 2 else float(x) * 1000 for i, x in enumerate(row)]
print(arr)
print(arr[0])
print(arr[1])
print(arr[2])
# print(type(arr))
# print(arr.shape)

desitrd_pose = [155, -263, 255]
print(desitrd_pose)
# print(type(desired_pose))
# print(desitrd_pose.shape)