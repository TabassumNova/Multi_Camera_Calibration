import sys 
import os 
# os.system('bash /opt/ros/dashing/setup.sh') # needs to be done in terminal before starting python 
import cv2 
import numpy as np 
import glob
import json 
import time 
import utils 

img = cv2.imread('C:/Users/FAPS/Desktop/Nova/camera_calib_nova/5MP_Cameras/cam743_images/p2.bmp')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#cv2.imshow('img',gray)
#cv2.waitKey(0)

# Find the chess board corners
#ret, corners = cv2.findCirclesGrid(gray, (3,3),None) 
ret, corners = cv2.findChessboardCorners(gray, (7,7), None)
# findChessboardCorners(gray, (7,7), None)
print(ret, corners)

#gray = np.float32(gray)
#dst = cv2.cornerHarris(gray,2,3,0.04)
#result is dilated for marking the corners, not important
#dst = cv2.dilate(dst,None)
# Threshold for an optimal value, it may vary depending on the image.
#img[dst>0.01*dst.max()]=[0,0,255]
#cv2.imshow('dst',img)
#if cv2.waitKey(0) & 0xff == 27:
#    cv2.destroyAllWindows()
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
corners2 = cv2.cornerSubPix(gray,corners,(4,4),(-1,-1),criteria)

# Draw and display the corners 
img = cv2.drawChessboardCorners(img, (4,4), corners2,ret) 
if(True): 
    #cv2.namedWindow('image',cv2.WINDOW_NORMAL) 
    im = cv2.resize(img, (1000,1000)) 
    cv2.imshow('img',im)
    #cv2.resizeWindow('image', (200,200)) 
    cv2.waitKey(0)