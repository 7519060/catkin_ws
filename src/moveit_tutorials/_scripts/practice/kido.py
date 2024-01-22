#!/usr/bin/env python
#coding: UTF-8
import cv2
import numpy as np

# img = cv2.imread('./desired_image.png', cv2.IMREAD_GRAYSCALE)
img = cv2.imread('./kensyo_desired_image.png', cv2.IMREAD_GRAYSCALE)
# img = cv2.imread('./kensyo_initial_image.png', cv2.IMREAD_GRAYSCALE)
def click_pos(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        img2 = np.copy(img)
        cv2.circle(img2, center=(x,y), radius=5, color=255, thickness=-1)
        brightness=img[y,x]
        bt_str = 'brightness='+str(brightness)
        cv2.putText(img2, bt_str,(10,30), cv2.FONT_HERSHEY_PLAIN,2,255,2,cv2.LINE_AA)
        cv2.imshow('window', img2)
        
cv2.imshow('window', img)
cv2.setMouseCallback('window', click_pos)
cv2.waitKey(0)
cv2.destroyAllWindows()
        

# print(max(img))