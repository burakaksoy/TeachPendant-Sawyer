import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

import cv2
import os
from os import listdir
from os.path import isfile, join

import opencv_camera_calibration as calibrator

class CameraCalibration_impl(object):
    def __init__(self):
        self.url_camera = None 
        self.camera_sub = None
        self.camera = None # RR camera object

        # Create a folder for saved images
        self.path = "./calibration_files"
        self.path_imgs = "./calibration_imgs"
        self.extension = ".yml"
        self.extension_imgs = ".png"
        self.prefix = "images"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

        if not os.path.exists(self.path_imgs):
            os.makedirs(self.path_imgs)

    def reset(self):
        self.url_camera = None 
        self.camera_sub = None
        self.camera = None # RR camera object

        # Create a folder for saved images
        self.path = "./calibration_files"
        self.path_imgs = "./calibration_imgs"
        self.extension = ".yml"
        self.extension_imgs = ".png"
        self.prefix = "images"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

        if not os.path.exists(self.path_imgs):
            os.makedirs(self.path_imgs)

    def connect2camera(self, url_camera):
        if self.camera is None:
            self.url_camera = url_camera

            self.camera = RRN.ConnectService(self.url_camera) # connect to camera with the given url
            # self.camera_sub = RRN.SubscriberService(self.url_camera)
            # self.camera = self.camera_sub.GetDefaultClientWait(1)

            # Define camera modes

            # log that the camera is successfully connected
            print("Camera is connected to CameraCalibration service!")
        else:
            # Give an error that says the camera is already connected
            print("Camera is already connected to CameraCalibration service! Trying to connect again..")
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

    def calibrate_n_save(self, filename, square_size, width, height):
        # filename: calibration file output filename
        # square_size: one side of the each square at the checkerboard in millimeters
        # width: number of squares in the checkerboard at one side
        # height: number of squares in the checkerboard at the other side

        # Calibrate
        ret, mtx, dist, rvecs, tvecs = calibrator.calibrate(self.path_imgs, self.prefix, self.extension_imgs, square_size, width-1,height-1)
        
        # And Save
        cv_file = cv2.FileStorage(self.path +"/"+ filename, cv2.FILE_STORAGE_WRITE)
        # Camera Matrix
        cv_file.write("K", mtx)
        # Distortion Coefficients
        cv_file.write("D", dist)
        # Rotation matrix 
        R_co, _ = cv2.Rodrigues(rvecs[0]) 
        cv_file.write("R_co", R_co )
        # Tranlastion vector
        cv_file.write("T_co", tvecs[0])
        # overall RMS re-projection error
        cv_file.write("error", ret)
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
            R_matrix = cv_file.getNode("R_co").mat()
            T_vector = cv_file.getNode("T_co").mat()
            error = cv_file.getNode("error").real()

            print(error)
            cv_file.release()

            # Pack the results into CalibrationParameters structure
            params = RRN.NewStructure("experimental.pluginCameraCalibration.CalibrationParameters")
            params.camera_matrix = camera_matrix
            params.distortion_coefficients = dist_matrix
            params.R_co = R_matrix
            params.T_co = T_vector
            params.error = error

            return params
        else:
            print("The file does not exist")

    def WebcamImageToMat(self, image):
        frame2=image.data.reshape([image.image_info.height, image.image_info.width, 3], order='C')
        return frame2

    def capture_image(self):
        if self.camera is not None:
            print("Capture_image function is called")
            try: 
                # Capture the current image from the camera and return
                frame = self.camera.capture_frame()
                # Get how many images exist in the imgs directory
                num_imgs = self.num_of_captured_images()
                # Set the image file name accordingly
                filename = self.prefix +str(num_imgs)+self.extension_imgs
                # Save it to the imgs directory
                cv2.imwrite(self.path_imgs+"/"+filename,self.WebcamImageToMat(frame))
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
        for f in os.listdir(self.path_imgs):
            if not f.endswith(self.extension_imgs):
                continue
            os.remove(os.path.join(self.path_imgs, f))

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-cameraCalibration-service", 8893) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginCameraCalibration")

        # create object
        CameraCalibration_inst = CameraCalibration_impl()
        # register service with service name "CameraCalibration", type "experimental.pluginCameraCalibration.CameraCalibration", actual object: CameraCalibration_inst
        RRN.RegisterService("CameraCalibration","experimental.pluginCameraCalibration.CameraCalibration",CameraCalibration_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginCameraCalibration Server started, press enter to quit...")
        else:
            raw_input("pluginCameraCalibration Server started, press enter to quit...")

if __name__ == '__main__':
    main()