#!/usr/bin/env python
#coding: UTF-8

import rospy
import numpy as np
import pandas as pd
import csv

def read():
    csvdata = pd.read_csv('./test.csv', header=None)
    print('csvyometa')
    pinv_int_mat = np.array
    pinv_int_mat = csvdata.values
    print('read csv file')
    print(pinv_int_mat.shape)
    
    csvdata2 = pd.read_csv('./test2.csv', header=None)
    print('csvyometa2')
    pinv_int_mat2 = np.array
    pinv_int_mat2 = csvdata2.values
    print('read csv file2')
    print(pinv_int_mat2.shape)
    
    I = np.dot(pinv_int_mat, pinv_int_mat2)
    print(I)
    
if __name__ == '__main__':
    read()