import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

import cv2



class CameraTraining_impl(object):
    def __init__(self):
        self.url_camera = None 
        self.camera_sub = None
        self.camera = None # RR camera object

    def reset(self):
        self.url_camera = None 
        self.camera_sub = None
        self.camera = None # RR camera object

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

    def WebcamImageToMat(self, image):
        frame2=image.data.reshape([image.image_info.height, image.image_info.width, 3], order='C')
        return frame2

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