#!/usr/bin/env python
#coding: UTF-8

import cv2 
import numpy as np
from matplotlib import pyplot as plt

# img = cv2.imread('./desired_image.png')
img = cv2.imread('./kensyo_desired_image.png')
# img = cv2.imread('./test_image.png', cv2.IMREAD_GRAYSCALE)

# hist, bins = np.histogram(img.flatten(), 256, [0, 256])
# print(hist.shape)

# total_pixels = img.shape[0]*img.shape[1]
# print(total_pixels)

# hist_percentage = hist / total_pixels
# print(hist_percentage.shape)

# plt.plot(bins[:-1], hist_percentage, color='b')
# plt.xlabel('luminace')
# plt.ylabel('frequency percentage')
# plt.grid(True)

# plt.plot(hist)

hist_cv = cv2.calcHist([img], [0], None, [256], [0, 256])

plt.plot(hist_cv)

plt.savefig('./hist.png')

plt.show()
# print(img[:,:,0])
# print(img[:,:,1])
# print(img[:,:,2])
###これでプリントしたら全部同じ行列出てた

#####UIカメラのカラーモードmono8だしimgmsg_to_cv2もオプションmono8にしてるけど
#####画像が3チャンネルで出力されてた
