import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

import cv2
import os
from os import listdir
from os.path import isfile, join
import json 

import general_robotics_toolbox as rox
import opencv_aruco_extrinsic_calibration as calibrator

class CameraRobotCalibration_impl(object):
    def __init__(self):
        self.url_camera = None 
        self.camera_sub = None
        self.camera = None # RR camera object

        # Create a folder for saved images
        self.path = "./calibration_files"
        self.path_imgs = "./calibration_imgs"
        self.path_poses = "./calibration_poses"
        self.extension = ".yml"
        self.extension_imgs = ".png"
        self.extension_poses = ".json"
        self.prefix = "images"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

        if not os.path.exists(self.path_imgs):
            os.makedirs(self.path_imgs)

        if not os.path.exists(self.path_poses):
            os.makedirs(self.path_poses)

        self.saved_endeff_poses_dict = {} # Dictionary to store corresponding end eff. pose RR Pose objects
        self.saved_marker_poses_dict = {} # Dictionary to store corresponding marker poses wrt robot

    def reset(self):
        self.url_camera = None 
        self.camera_sub = None
        self.camera = None # RR camera object

        # Create a folder for saved images
        self.path = "./calibration_files"
        self.path_imgs = "./calibration_imgs"
        self.path_poses = "./calibration_poses"
        self.extension = ".yml"
        self.extension_imgs = ".png"
        self.extension_poses = ".json"
        self.prefix = "images"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

        if not os.path.exists(self.path_imgs):
            os.makedirs(self.path_imgs)

        if not os.path.exists(self.path_poses):
            os.makedirs(self.path_poses)

        self.saved_endeff_poses_dict = {} # Dictionary to store corresponding end eff. pose RR Pose objects
        self.saved_marker_poses_dict = {} # Dictionary to store corresponding marker poses wrt robot

    def connect2camera(self, url_camera):
        if self.camera is None:
            self.url_camera = url_camera

            self.camera = RRN.ConnectService(self.url_camera) # connect to camera with the given url
            # self.camera_sub = RRN.SubscriberService(self.url_camera)
            # self.camera = self.camera_sub.GetDefaultClientWait(1)

            # Define camera modes

            # log that the camera is successfully connected
            print("Camera is connected to CameraRobotCalibration service!")
        else:
            # Give an error that says the camera is already connected
            print("Camera is already connected to CameraRobotCalibration service! Trying to connect again..")
            self.reset()
            self.connect2camera(url_camera)

    def saved_calibrations(self):
        self.saved_calibration_filenames = [f for f in listdir(self.path) if f.endswith(self.extension)]

        print(self.saved_calibration_filenames)
        return self.saved_calibration_filenames

    def delete_calibration(self,filename):
        if os.path.exists(self.path +"/"+ filename):
            os.remove(self.path +"/"+ filename)
        else:
            print("The file does not exist")

    def calibrate_n_save(self, filename, aruco_dict, aruco_id, aruco_markersize, T_eef_to_marker,R_RPY_eef_to_marker, mtx, dist):
        # filename: calibration file output filename
        # mtx: opencv camera matrix 
        # dist: opencv distortion coefficients

        ## Add marker pose to the end effector poses and create a dict for marker poses wrt robot base
        # Convert R_RPY euler angles to rotation matrix
        Rx = rox.rot(np.array(([1.],[0.],[0.])), np.deg2rad(float(R_RPY_eef_to_marker[0])))
        Ry = rox.rot(np.array(([0.],[1.],[0.])), np.deg2rad(float(R_RPY_eef_to_marker[1])))
        Rz = rox.rot(np.array(([0.],[0.],[1.])), np.deg2rad(float(R_RPY_eef_to_marker[2])))
        R_eef_to_marker = Rz @ Ry @ Rx

        # Convert given T_eef_to_marker to mm from meters
        T_eef_to_marker = T_eef_to_marker*1000.0

        for key in self.saved_endeff_poses_dict:
            R_base_to_eef = self.saved_endeff_poses_dict[key][0]
            R_base_to_marker = R_base_to_eef @ R_eef_to_marker

            T_base_to_eef = self.saved_endeff_poses_dict[key][1]
            T_base_to_marker = T_base_to_eef + (R_base_to_eef @ T_eef_to_marker)
            self.saved_marker_poses_dict[key] = [R_base_to_marker.tolist(),T_base_to_marker.tolist()] 
            # Note that now dictonary became for saved marker poses with list elements rather than np.arrays

        # Save the end effector poses dictonary to a json file
        saved_poses_filename = filename.split(self.extension)[0]+self.extension_poses # replaces .yml extension with .json
        
        if len(self.saved_endeff_poses_dict) > 0: 
            with open(self.path_poses+'/'+saved_poses_filename, 'w') as fp:
                json.dump(self.saved_marker_poses_dict, fp, sort_keys=True)

        # Calibrate
        R_cam2base, T_cam2base, R_base2cam, T_base2cam = calibrator.calibrate(self.path_imgs, self.prefix, self.extension_imgs, self.path_poses, saved_poses_filename, aruco_dict, aruco_id, aruco_markersize, mtx, dist)
        
        # And Save
        cv_file = cv2.FileStorage(self.path +"/"+ filename, cv2.FILE_STORAGE_WRITE)
        # Camera Matrix
        cv_file.write("K", mtx)
        # Distortion Coefficients
        cv_file.write("D", dist)
        # Rotation matrix 
        cv_file.write("R_co", R_cam2base )
        # Tranlastion vector
        cv_file.write("T_co", T_cam2base)
        # Rotation matrix 
        cv_file.write("R_oc", R_base2cam )
        # Tranlastion vector
        cv_file.write("T_oc", T_base2cam)
        # note you *release* you don't close() a FileStorage object
        cv_file.release()


    def load_calibration(self, filename):
        if os.path.exists(self.path +"/"+ filename):
            # FILE_STORAGE_READ
            cv_file = cv2.FileStorage(self.path +"/"+ filename, cv2.FILE_STORAGE_READ)

            # note we also have to specify the type to retrieve other wise we only get a
            # FileNode object back instead of a matrix
            camera_matrix = cv_file.getNode("K").mat()
            dist_matrix = cv_file.getNode("D").mat()
            R_matrix_co = cv_file.getNode("R_co").mat()
            T_vector_co = cv_file.getNode("T_co").mat()
            R_matrix_oc = cv_file.getNode("R_oc").mat()
            T_vector_oc = cv_file.getNode("T_oc").mat()

            cv_file.release()

            # Pack the results into CalibrationExtrinsicParameters structure
            params = RRN.NewStructure("experimental.pluginCameraRobotCalibration.CalibrationExtrinsicParameters")
            params.camera_matrix = camera_matrix
            params.distortion_coefficients = dist_matrix
            params.R_co = R_matrix_co
            params.T_co = T_vector_co
            params.R_oc = R_matrix_oc
            params.T_oc = T_vector_oc

            return params
        else:
            print("The file does not exist")

    def WebcamImageToMat(self, image):
        frame2=image.data.reshape([image.image_info.height, image.image_info.width, 3], order='C')
        return frame2

    def capture_image_with_robot_pose(self, R,T):
        if self.camera is not None:
            print("capture_image_with_robot_pose function is called")
            try: 
                # Capture the current image from the camera and return
                frame = self.camera.capture_frame()
                # Get how many images exist in the imgs directory
                num_imgs = self.num_of_captured_images()
                # Set the image file name accordingly
                filename = self.prefix +str(num_imgs)+self.extension_imgs
                # Save it to the imgs directory
                cv2.imwrite(self.path_imgs+"/"+filename,self.WebcamImageToMat(frame))

                # Add the current robot pose into poses dictionary for the captured image
                self.saved_endeff_poses_dict[filename] = [R,T*1000.0]  # robot_pose is a RR Pose object

            except:
                import traceback
                print(traceback.format_exc())
            
        else:
            # Give an error message to show that the robot is not connected
            print("Image capturing failed. Camera is not connected to Cameracalibration service yet!")

    def num_of_captured_images(self):
        self.saved_images_filenames = [f for f in listdir(self.path_imgs) if f.endswith(self.extension_imgs)]

        # print(len(self.saved_images_filenames))
        return len(self.saved_images_filenames)

    def remove_captured_images(self):
        # Remove images
        for f in os.listdir(self.path_imgs):
            if not f.endswith(self.extension_imgs):
                continue
            os.remove(os.path.join(self.path_imgs, f))

        # Remove corresponding poses json files
        for f in os.listdir(self.path_poses):
            if not f.endswith(self.extension_poses):
                continue
            os.remove(os.path.join(self.path_poses, f))

        # Also reset the corresponding poses dict
        self.saved_endeff_poses_dict = {}
        self.saved_marker_poses_dict = {}

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-cameraRobotCalibration-service", 8902) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginCameraRobotCalibration")

        # create object
        CameraRobotCalibration_inst = CameraRobotCalibration_impl()
        # register service with service name "CameraCalibration", type "experimental.pluginCameraCalibration.CameraCalibration", actual object: CameraCalibration_inst
        RRN.RegisterService("CameraRobotCalibration","experimental.pluginCameraRobotCalibration.CameraRobotCalibration",CameraRobotCalibration_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginCameraRobotCalibration Server started, press enter to quit...")
        else:
            raw_input("pluginCameraRobotCalibration Server started, press enter to quit...")

if __name__ == '__main__':
    main()