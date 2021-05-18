#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 29 23:56:17 2021

@author: burak
"""

import cv2
import numpy as np

orig_point = np.array([0.5,0.2,2])

print(orig_point)

tvec_cam = np.array([0,0,1.])
rvec_cam = np.array([0,0,0.])

mtx = np.array([[600,0,200],[0,600,200],[0,0,1]],dtype=np.float64)
dist = np.array([0,0,0,0,0],dtype=np.float64)

img_pts = cv2.projectPoints(orig_point, rvec_cam, tvec_cam, mtx, dist)[0]

#dst = cv2.undistortPoints(,mtx,dist)
dst = cv2.undistortPoints(img_pts,mtx,dist) # dst is Xc/Zc and Yc/Zc in the same shape of src
dst = dst * float(tvec_cam[2]+orig_point[2]) * 1000.0 # Multiply by given Zc distance to find all cordinates, multiply by 1000 is because of Zc is given in meters but others are in millimeters
dst = np.squeeze(dst) * 0.001 # Xc and Yc as vector

print(dst)