#!/usr/bin/env python 
#coding: UTF-8

from PIL import Image
import numpy as np
import cv2
color_image = cv2.imread('./data/20221202_215837_image.png')
# print('出力画像のサイズ')
# print(color_image.shape)
# ###opencv2_tutorial.py deha 3channel datta
# ###koreno kekka ha (240,320,3) reshape no ato (230400,1)datta
im_gray = cv2.imread('./data/20221202_215837_image.png', cv2.IMREAD_GRAYSCALE)
# #imreadのoptionでグレーにした画像を保存
# cv2.imwrite('./data/grayscale_215837_image.png', im_gray)
# print('imreadのcv2.IMREAD_GRAYSCALEで変換した画像のサイズ')
# print(im_gray.shape)

# #column vector
# im_c_vec_gray = im_gray.reshape(-1,1)
# print('列ベクトルに直した画像のサイズ')
# print(im_c_vec_gray.shape)

# ###cv2,IMREAD_GRAYSCALE de 1channel toshite yomikometa 
# ###koreno kekka ha (240,320) reshape no ato (76800,1) datta 

# ###opencv_tutorial2.py de 3 channel gray image ni suru hitsuyou ha nasasou
# ###opencv_tutorial2.pu deha color image de hozon suru

# resize_again = cv2.resize(color_image, None, fx = 0.5, fy = 0.5)
# print('capture_serverでリサイズはしてあるが更にリサイズした画像のサイズ')
# print(resize_again.shape)

# GRAY = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
# print('cvtColorでBGR2GRAYのサイズ')
# print(GRAY.shape)
# BGR = cv2.cvtColor(GRAY, cv2.COLOR_GRAY2BGR)
# print('cvtColorでGRAY2BGRのサイズ')
# print(BGR.shape)

# retsuvec = BGR.reshape(-1,1)
# print(retsuvec.shape)

###行列表示できるか確認
# print(color_image)
print(im_gray)
