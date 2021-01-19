import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

import cv2
import math 

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

        self._image_consts = RRN.GetConstants('com.robotraconteur.image')
        self._image_info_type = RRN.GetStructureType('com.robotraconteur.image.ImageInfo')
        self._compressed_image_type = RRN.GetStructureType('com.robotraconteur.image.CompressedImage')

        
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

    def _cv_mat_to_compressed_image(self, mat, quality = 100):
        is_mono = False
        if (len(mat.shape) == 2 or mat.shape[2] == 1):
            is_mono = True

        image_info = self._image_info_type()
        image_info.width =mat.shape[1]
        image_info.height = mat.shape[0]
        
        image_info.step = 0
        image_info.encoding = self._image_consts["ImageEncoding"]["compressed"]
        
        image = self._compressed_image_type()
        image.image_info = image_info
        res, encimg = cv2.imencode(".jpg",mat,[int(cv2.IMWRITE_JPEG_QUALITY), quality])
        assert res, "Could not compress frame!"
        image.data=encimg
        return image

    def find_object_in_img_frame(self, obj_img_filename, camera_name, return_result_image):
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
            # return_result_image = True
            if return_result_image:
                self.center, self.wh, self.angle, self.detection_result_img = matcher.detect_object(return_result_image)
            else:
                self.center, self.wh, self.angle = matcher.detect_object()
                self.detection_result_img = None
            
            # return the pose of the object
            print("the object is found..:")
            print("center coordinates in img frame(x,y): " + str(self.center))
            print("(w,h): " + str(self.wh))
            print("angle: " + str(self.angle))


            # Pack the results into DetectedObject structure
            detection_result = RRN.NewStructure("experimental.pluginCameraTracking.DetectedObject")
            detection_result.width = self.wh[0]
            detection_result.height = self.wh[1]
            detection_result.center_x = self.center[0]
            detection_result.center_y = self.center[1]
            detection_result.angle = self.angle

            if self.detection_result_img is not None:
                # Convert opencv result image to compressed RR image
                detection_result.result_img = self._cv_mat_to_compressed_image(self.detection_result_img, 70)
            else:
                detection_result.result_img = None

            return detection_result

        else:
            # Give an error message to show that the robot is not connected
            print("Cameras or plugins are not connected to CameraTracking service yet!")

    def find_object_pose_in_cam_frame(self,obj_img_filename, camera_name, value_z_distance):
        if self.is_plugins_connected and self.is_cameras_connected:
            # If the value of z distance is given bigger than zero use the given one 
            #otherwise camera has 3D capability, get z value from thecapability of the camera
            if value_z_distance <= 0:
                #TODO
                # value_z_distance = getFrom3DCapableCamera
                print("Provide a proper positive z distance")
            
            # Find the object in image frame 
            detection_result = self.find_object_in_img_frame(obj_img_filename, camera_name, False)
            
            # detection_result.width
            # detection_result.height
            x = detection_result.center_x
            y = detection_result.center_y
            theta = detection_result.angle
            src = np.asarray([x,y], dtype=np.float32)
            src = np.reshape(src,(-1,1,2)) # Rehsape as opencv requires (N,1,2)
            # print(src)
            # Now the detection results in image frame is found,
            # Hence, find the detected pose in camera frame with given z distance to camera
            # To do that first we need to get the camera parameters
            # Load the camera matrix and distortion coefficients from the calibration result.
            filename = str(camera_name) + ".yml"
            params = self.plugin_cameraCalibration.load_calibration(filename)

            mtx = params.camera_matrix 
            dist = params.distortion_coefficients
            # R_co = params.R_co 
            # T_co = params.T_co 
            # error = params.error

            # Find the corresponding world pose of the detected pose in camera frame
            dst = cv2.undistortPoints(src,mtx,dist) # dst is Xc/Zc and Yc/Zc in the same shape of src
            dst = dst * float(value_z_distance) * 1000.0 # Multiply by given Zc distance to find all cordinates, multiply by 1000 is because of Zc is given in meters but others are in millimeters
            dst = np.squeeze(dst) * 0.001 # Xc and Yc as vector

            # Finally the translation between the detected object center and the camera frame represented in camera frame is T = [Xc,Yc,Zc]
            Xc = dst[0]
            Yc = dst[1]
            Zc = float(value_z_distance)
            T = np.asarray([Xc,Yc,Zc])

            # Now lets find the orientation of the detected object with respect to camera 
            # We are assuming +z axis is looking towards the camera and xy axes of the both object and camera are parallel planes
            # So the rotation matrix would be
            theta = np.deg2rad(theta) #convert theta from degrees to radian
            R_co = np.asarray([[math.cos(theta),-math.sin(theta),0],[-math.sin(theta),-math.cos(theta),0],[0,0,-1]])

            # Pack the results into CalibrationParameters structure
            pose = RRN.NewStructure("experimental.pluginCameraTracking.Pose")
            pose.R = R_co
            pose.T = T

            return pose

        else:
            # Give an error message to show that the robot is not connected
            print("Cameras or plugins are not connected to CameraTracking service yet!")




def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-cameraTracking-service", 8898) as node_setup:

        # register service type
        # RRN.RegisterServiceTypeFromFile("./experimental.pluginCameraTracking")
        RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.image',"./experimental.pluginCameraTracking"],True)

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