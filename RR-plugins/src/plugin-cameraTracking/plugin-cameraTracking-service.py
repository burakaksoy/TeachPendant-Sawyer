import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

class CameraTracking_impl(object):
    def __init__(self):
        self.url_plugins_vision_lst = [] # The order will be :
        # url_plugin_cameraFeedback, url_plugin_cameraTraining, url_plugin_cameraCalibration
        self.plugin_cameraFeedback = None
        self.plugin_cameraTraining = None
        self.plugin_cameraCalibration = None

        self.camera_name_url_dict = {} # Dictionary for connected camera urls(key:node names)
        self.camera_objs_dict = {} # Dictionary for connected camera objects(key:node names)
        
    def reset_vision_plugins(self):
        self.url_plugins_vision_lst = [] 
        
        self.plugin_cameraFeedback = None
        self.plugin_cameraTraining = None
        self.plugin_cameraCalibration = None
        self.plugin_cameraTracking = None    

    def reset_connected_cameras(self):
        self.camera_name_url_dict = {} # Dictionary for connected camera urls(key:node names)
        self.camera_objs_dict = {} # Dictionary for connected camera objects(key:node names)

    # Make camera tracking connect to all vision plugins as well so that it can reach the inner files of those services with their permit
    def connect2plugins_vision(self, url_plugins_vision_lst):
        if not self.url_plugins_vision_lst: # if the list is empty
            self.url_plugins_vision_lst = url_plugins_vision_lst # append the new urls
            # self.url_plugins_vision_lst = list(set(self.url_plugins_vision_lst)) # keep only the unique urls, prevent adding the same urls again
            print("vision plugin urls:")
            print(self.url_plugins_vision_lst)

            self.plugin_cameraFeedback = RRN.ConnectService(self.url_plugins_vision_lst[0])
            self.plugin_cameraTraining = RRN.ConnectService(self.url_plugins_vision_lst[1])
            self.plugin_cameraCalibration = RRN.ConnectService(self.url_plugins_vision_lst[2])
        else:
            # Give an error that says the vision plugins are already connected
            print("Vision plugins are already connected to CameraTracking service! Trying to connect again..")
            self.reset_vision_plugins()
            self.connect2plugins_vision(url_plugins_vision_lst)

    # Make tracking plugin to connect to all cameras and make it get the all corresponding camera (node) names
    def connect2all_cameras(self, camera_connection_urls_lst, camera_node_names_lst):
        if not self.camera_objs_dict: # if the dictionary is empty
            self.camera_name_url_dict = dict(zip(camera_node_names_lst,camera_connection_urls_lst)) 

            for camera_name, camera_url in self.camera_name_url_dict.items():
                # connect to the camera service url
                camera_obj = RRN.ConnectService(camera_url) # connect to cam with given url
                # add the connected camera object to camera object dictionary
                self.camera_objs_dict[camera_name] = camera_obj

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
        if self.camera_objs_dict and self.plugin_cameraCalibration is not None :
            # TODO: Later it can be used for storing camera parameters etc
            pass
        else:
            # Give an error message to show that the robot is not connected
            print("Assign camera details failed. cameras are not connected to CameraTracking service yet!")


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