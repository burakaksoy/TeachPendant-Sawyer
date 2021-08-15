import cv2
import numpy as np
import os
from os import listdir
from os.path import isfile, join

import opencv_aruco_extrinsic_calibration as calibrator

#%% Test 1 Kinect and sawyer robot in lab
path_imgs = "./backup-sawyer-kinect-lab/calibration_imgs"
prefix= "images"
extension_imgs =".png"
path_poses="./backup-sawyer-kinect-lab/calibration_poses"

saved_poses_filename="sawyer_robot--com.robotraconteur.imaging.camera.json" # Change this as "robotname--cameraname.json"
aruco_dict="cv2.aruco.DICT_ARUCO_ORIGINAL" 
aruco_id = 7
aruco_markersize = 70.0 # in mm

calibration_filename = "com.robotraconteur.imaging.camera.yml"
calibration_filepath = "../plugin-cameraCalibration/calibration_files/"

if os.path.exists(calibration_filepath + calibration_filename):
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(calibration_filepath + calibration_filename, cv2.FILE_STORAGE_READ)
    
    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    mtx = cv_file.getNode("K").mat()
    dist = cv_file.getNode("D").mat()

R_cam2base,T_cam2base,R_base2cam,T_base2cam = calibrator.calibrate(path_imgs, prefix, extension_imgs, path_poses, saved_poses_filename, aruco_dict, aruco_id, aruco_markersize, mtx, dist)

#T0_c = np.asarray([0.6659,-0.8237,1.1862], dtype=np.float32) # Predefined when camera is fixed (LAB KINECT)
## T_base2cam/1000 = array([ 0.64985101, -0.80554785,  1.13488919])

# R0_c = np.asarray([[0,-1,0],[-1,0,0],[0,0,-1]], dtype=np.float32) # Predefined when camera is fixed (LAB KINECT)

#%% Test 2 for simulation camera with sawyer

path_imgs = "./backup-sawyer-cam1-simulation/calibration_imgs"
prefix= "images"
extension_imgs =".png"
path_poses="./backup-sawyer-cam1-simulation/calibration_poses"

saved_poses_filename="sawyer_robot--camera_sim1.json" # Change this as "robotname--cameraname.json"
aruco_dict="cv2.aruco.DICT_ARUCO_ORIGINAL" 
aruco_id = 7
aruco_markersize = 50.0 # in mm

calibration_filename = "camera_sim1.yml"
calibration_filepath = "../plugin-cameraCalibration/calibration_files/"

if os.path.exists(calibration_filepath + calibration_filename):
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(calibration_filepath + calibration_filename, cv2.FILE_STORAGE_READ)
    
    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    mtx = cv_file.getNode("K").mat()
    dist = cv_file.getNode("D").mat()

R_cam2base,T_cam2base,R_base2cam,T_base2cam = calibrator.calibrate(path_imgs, prefix, extension_imgs, path_poses, saved_poses_filename, aruco_dict, aruco_id, aruco_markersize, mtx, dist)

# T0_c = np.asarray([0.5,0,0.73], dtype=np.float32) # Predefined when camera is fixed
# R0_c = np.asarray([[0,1,0],[1,0,0],[0,0,-1]], dtype=np.float32) # Predefined when camera is fixed