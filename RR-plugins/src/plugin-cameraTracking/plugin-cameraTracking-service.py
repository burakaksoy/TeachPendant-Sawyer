import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

import cv2

from opencv_template_matching import TemplateMatchingMultiAngle

class CameraTracking_impl(object):
    def __init__(self):
        self.url_plugins_vision_lst = [] # The order will be :
        # url_plugin_cameraFeedback, url_plugin_cameraTraining, url_plugin_cameraCalibration
        self.plugin_cameraFeedback = None
        self.plugin_cameraTraining = None
        self.plugin_cameraCalibration = None
        self.is_plugins_connected = False

        self.camera_name_url_dict = {} # Dictionary for connected camera urls(key:node names)
        self.camera_objs_dict = {} # Dictionary for connected camera objects(key:node names)
        self.is_cameras_connected = False
        
    def reset_vision_plugins(self):
        self.url_plugins_vision_lst = [] 
        
        self.plugin_cameraFeedback = None
        self.plugin_cameraTraining = None
        self.plugin_cameraCalibration = None
        self.plugin_cameraTracking = None    
        self.is_plugins_connected = False

    def reset_connected_cameras(self):
        self.camera_name_url_dict = {} # Dictionary for connected camera urls(key:node names)
        self.camera_objs_dict = {} # Dictionary for connected camera objects(key:node names)
        self.is_cameras_connected = False

    # Make camera tracking connect to all vision plugins as well so that it can reach the inner files of those services with their permit
    def connect2plugins_vision(self, url_plugins_vision_lst):
        if not self.is_plugins_connected: # if the list is empty
            self.url_plugins_vision_lst = url_plugins_vision_lst # append the new urls
            # self.url_plugins_vision_lst = list(set(self.url_plugins_vision_lst)) # keep only the unique urls, prevent adding the same urls again
            print("vision plugin urls:")
            print(self.url_plugins_vision_lst)

            self.plugin_cameraFeedback = RRN.ConnectService(self.url_plugins_vision_lst[0])
            self.plugin_cameraTraining = RRN.ConnectService(self.url_plugins_vision_lst[1])
            self.plugin_cameraCalibration = RRN.ConnectService(self.url_plugins_vision_lst[2])
            self.is_plugins_connected = True
        else:
            # Give an error that says the vision plugins are already connected
            print("Vision plugins are already connected to CameraTracking service! Trying to connect again..")
            self.reset_vision_plugins()
            self.connect2plugins_vision(url_plugins_vision_lst)

    # Make tracking plugin to connect to all cameras and make it get the all corresponding camera (node) names
    def connect2all_cameras(self, camera_connection_urls_lst, camera_node_names_lst):
        if not self.is_cameras_connected: # if the dictionary is empty
            self.camera_name_url_dict = dict(zip(camera_node_names_lst,camera_connection_urls_lst)) 

            for camera_name, camera_url in self.camera_name_url_dict.items():
                # connect to the camera service url
                camera_obj = RRN.ConnectService(camera_url) # connect to cam with given url
                # add the connected camera object to camera object dictionary
                self.camera_objs_dict[camera_name] = camera_obj

            self.is_cameras_connected = True
            # log that the cameras are successfully connected
            print("All cameras are connected to CameraTracking service!")

            # TODO: For assigning camera parameters etc
            self.assign_camera_details()

        else:
            # Give an error that says the vision plugins are already connected
            print("Cameras are already connected to CameraTracking service! Trying to connect again..")
            self.reset_connected_cameras()
            self.connect2all_cameras(camera_connection_urls_lst, camera_node_names_lst)

    def assign_camera_details(self):
        if self.is_plugins_connected and self.is_cameras_connected :
            # TODO: Later it can be used for storing camera parameters etc
            pass
        else:
            # Give an error message to show that the robot is not connected
            print("Assign camera details failed. Cameras or plugins are not connected to CameraTracking service yet!")

    def WebcamImageToMat(self, image):
        frame2=image.data.reshape([image.image_info.height, image.image_info.width, 3], order='C')
        return frame2

    def find_object_in_img_frame(self, obj_img_filename, camera_name):
        if self.is_plugins_connected and self.is_cameras_connected :
            # print("We are in DEBUG")

            # Load img from the given filename
            # Use cameraTraining plugin image load function
            img_obj = self.plugin_cameraTraining.load_image(obj_img_filename) # this returns RR image object
            img_obj = self.WebcamImageToMat(img_obj) # convert RR image object to cv image object

            # #Show the filed template image
            # cv2.imshow(obj_img_filename,img_obj)
            # cv2.waitKey(0)

            # capture image from the given camera_name 
            camera = self.camera_objs_dict[camera_name]# camera object
            img_compressed_cam = camera.capture_frame_compressed() # get the camera img as RR image
            img_compressed_cam = cv2.imdecode(img_compressed_cam.data,1) # convert it to cv image

            # cv2.imshow(camera_name,img_compressed_cam)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            # get the camera parameters from camera calibration (later) TODO

            # execute the image detection using opencv
            matcher = TemplateMatchingMultiAngle(img_obj,img_compressed_cam)
            center, wh, angle = matcher.detect_object()
            
            # return the pose of the object
            print("the object is found..:")
            print("center coordinates in img frame: " + str(center))
            print("(w,h): " + str(wh))
            print("angle: " + str(angle))
            
        else:
            # Give an error message to show that the robot is not connected
            print("Cameras or plugins are not connected to CameraTracking service yet!")



def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-cameraTracking-service", 8898) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginCameraTracking")

        # create object
        CameraTracking_inst = CameraTracking_impl()
        # register service with service name "CameraTracking", type "experimental.pluginCameraTracking.CameraTracking", actual object: CameraTracking_inst
        RRN.RegisterService("CameraTracking","experimental.pluginCameraTracking.CameraTracking",CameraTracking_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginCameraTracking Server started, press enter to quit...")
        else:
            raw_input("pluginCameraTracking Server started, press enter to quit...")

if __name__ == '__main__':
    main()