import sys
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

import cv2
import os
from os import listdir
from os.path import isfile, join
import base64


class CameraTraining_impl(object):
    def __init__(self):
        self._image_consts = RRN.GetConstants('com.robotraconteur.image')
        self._image_type = RRN.GetStructureType('com.robotraconteur.image.Image')
        self._image_info_type = RRN.GetStructureType('com.robotraconteur.image.ImageInfo')

        self.url_camera = None 
        self.camera_sub = None
        self.camera = None # RR camera object

        # Create a folder for saved images
        self.path = "./savedObjectImgs"
        self.extension = ".png"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

    def reset(self):
        self.url_camera = None 
        self.camera_sub = None
        self.camera = None # RR camera object

        # Create a folder for saved images
        self.path = "./savedObjectImgs"
        self.extension = ".png"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

    def connect2camera(self, url_camera):
        if self.camera is None:
            self.url_camera = url_camera

            self.camera = RRN.ConnectService(self.url_camera) # connect to robot with the given url
            # self.camera_sub = RRN.SubscriberService(self.url_camera)
            # self.camera = self.camera_sub.GetDefaultClientWait(1)

            # Define camera modes

            self.assign_camera_details()

            # log that the camera is successfully connected
            print("Camera is connected to CameraTraining service!")
        else:
            # Give an error that says the camera is already connected
            print("Camera is already connected to CameraTraining service! Trying to connect again..")
            self.reset()
            self.connect2camera(url_camera)

    def assign_camera_details(self):
        if self.camera is not None:
            # TODO: Later it can be used for storing camera parameters etc
            pass
        else:
            # Give an error message to show that the robot is not connected
            print("Assign camera details failed. camera is not connected to CameraTraining service yet!")

    def train_new_visual(self):
        if self.camera is not None:
            print("train_new_visual function is called")
            try: 
                # Capture the current image from the camera and return
                frame = self.camera.capture_frame()

                # while True:
                #     # compressed_image = self.camera.capture_frame_compressed()
                #     frame = self.camera.capture_frame()

                #     # frame2 = cv2.imdecode(compressed_image.data,1)
                #     frame2=self.WebcamImageToMat(frame)

                #     cv2.imshow("camera",frame2)
                #     cv2.waitKey(1)
                # cv2.destroyAllWindows()

                return frame
            except:
                import traceback
                print(traceback.format_exc())
            
        else:
            # Give an error message to show that the robot is not connected
            print("Train New Visual failed. camera is not connected to CameraTraining service yet!")

    def saved_images(self):
        # self.saved_workspaces_filenames = [f for f in listdir(self.path) if isfile(join(self.path, f))]
        self.saved_images_filenames = [f for f in listdir(self.path) if f.endswith(self.extension)]

        print(self.saved_images_filenames)
        return self.saved_images_filenames

    def _cv_mat_to_image(self, mat):

        is_mono = False
        if (len(mat.shape) == 2 or mat.shape[2] == 1):
            is_mono = True

        image_info = self._image_info_type()
        image_info.width =mat.shape[1]
        image_info.height = mat.shape[0]
        if is_mono:
            image_info.step = mat.shape[1]
            image_info.encoding = self._image_consts["ImageEncoding"]["mono8"]
        else:
            image_info.step = mat.shape[1]*3
            image_info.encoding = self._image_consts["ImageEncoding"]["rgb8"]

        image = self._image_type()
        image.image_info = image_info
        image.data=mat.reshape(mat.size, order='C')
        return image

    def load_image(self, filename):
        if os.path.exists(self.path +"/"+ filename):
            img = cv2.imread(self.path +"/"+ filename)
            return self._cv_mat_to_image(img)
        else:
            print("The file does not exist")

    def delete_image(self,filename):
        if os.path.exists(self.path +"/"+ filename):
            os.remove(self.path +"/"+ filename)
        else:
            print("The file does not exist")

    def edit_image_name(self, filename, file_name_new):
        if os.path.exists(self.path +"/"+ filename):
            os.rename(self.path +"/"+ filename, self.path +"/"+ file_name_new) 
        else:
            print("The file does not exist")

    def WebcamImageToMat(self, image):
        frame2=image.data.reshape([image.image_info.height, image.image_info.width, 3], order='C')
        return frame2

    def save_image(self, filename, img_str, width, height):
        if not os.path.exists(self.path +"/"+ filename):
            try:
                # convert String image to cv frame 
                imageBytes = base64.b64decode(img_str)
                # print("imageBytes22222222222")
                # print(imageBytes)
                # print("len(imageBytes)2222222222")
                # print(len(imageBytes))

                nparr = np.frombuffer(imageBytes, np.uint8)
                img = cv2.imdecode(nparr, 1)  
                print(img.shape)

                # cv2.imshow("img_decode", img)
                # cv2.waitKey()
                # print("HERE222")

                cv2.imwrite(os.path.join(self.path , filename), img)
                cv2.waitKey(0)
            except:
                import traceback
                print(traceback.format_exc())
                pass
        else:
            print("The file already exists!")


def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-cameraTraining-service", 8892) as node_setup:

        # register service type
        # RRN.RegisterServiceTypeFromFile("./experimental.pluginCameraTraining")
        RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.image',"./experimental.pluginCameraTraining"],True)

        # create object
        CameraTraining_inst = CameraTraining_impl()
        # register service with service name "CameraTraining", type "experimental.pluginCameraTraining.CameraTraining", actual object: CameraFeedback_inst
        RRN.RegisterService("CameraTraining","experimental.pluginCameraTraining.CameraTraining",CameraTraining_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginCameraTrainingServer started, press enter to quit...")
        else:
            raw_input("pluginCameraTraining Server started, press enter to quit...")

if __name__ == '__main__':
    main()