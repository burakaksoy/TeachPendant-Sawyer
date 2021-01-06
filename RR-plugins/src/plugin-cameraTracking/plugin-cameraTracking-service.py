import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

import cv2
import math
from matplotlib import pyplot as plt

class RRect_center:
  def __init__(self, p0, s, S, ang):
    (self.W, self.H) = s # rotated image width and height
    (self.w, self.h) = S # original rectangle width and height
    self.d = math.sqrt(self.w**2 + self.h**2)/2.0 # distance from center to vertices    
    self.c = (int(p0[0]+self.W/2.0),int(p0[1]+self.H/2.0)) # center point coordinates
    self.ang = ang # rotation angle
    self.alpha = math.radians(self.ang) # rotation angle in radians
    self.beta = math.atan2(self.h, self.w) # angle between d and horizontal axis
    # Center Rotated vertices in image frame
    self.P0 = (int(self.c[0] - self.d * math.cos(self.beta - self.alpha)), int(self.c[1] - self.d * math.sin(self.beta-self.alpha))) 
    self.P1 = (int(self.c[0] - self.d * math.cos(self.beta + self.alpha)), int(self.c[1] + self.d * math.sin(self.beta+self.alpha))) 
    self.P2 = (int(self.c[0] + self.d * math.cos(self.beta - self.alpha)), int(self.c[1] + self.d * math.sin(self.beta-self.alpha))) 
    self.P3 = (int(self.c[0] + self.d * math.cos(self.beta + self.alpha)), int(self.c[1] - self.d * math.sin(self.beta+self.alpha))) 

    self.verts = [self.P0,self.P1,self.P2,self.P3]

  def draw(self, image):
    # print(self.verts)
    blue  = (255,0  ,0  )
    yellow= (0  ,255,255)
    green = (0  ,255,0  )
    red   = (0  ,0  ,255)
    colors = [red,yellow,green,blue]
    
    for i in range(len(self.verts)-1):
      # cv2.line(image, (self.verts[i][0], self.verts[i][1]), (self.verts[i+1][0],self.verts[i+1][1]), (0,255,0), 2)
      cv2.line(image, (self.verts[i][0], self.verts[i][1]), (self.verts[i+1][0],self.verts[i+1][1]), colors[i], 2)
    # cv2.line(image, (self.verts[3][0], self.verts[3][1]), (self.verts[0][0], self.verts[0][1]), (0,255,0), 2)
    cv2.line(image, (self.verts[3][0], self.verts[3][1]), (self.verts[0][0], self.verts[0][1]), colors[3], 2)

class TemplateMatchingMultiAngle(object):
    def __init__(self, template_img, camera_img):
        self.template_img = template_img
        self.camera_img = camera_img

        self.max_angle = 180.0 # degree
        self.min_angle = -180.0 # degree
        self.num_angles = 19 # number of different angles (has to be integer)

        # There are 6 Template Matching methods 
        # self.method = cv2.TM_CCOEFF
        # self.method = cv2.TM_CCOEFF_NORMED 
        self.method = cv2.TM_CCORR # Seems to be best w/out canny
        # self.method = cv2.TM_CCORR_NORMED
        # self.method = cv2.TM_SQDIFF
        # self.method = cv2.TM_SQDIFF_NORMED

    

    def rotate_bound(self, image, angle):
        # grab the dimensions of the image and then determine the
        # center
        (h, w) = image.shape[:2]
        (cX, cY) = (w / 2, h / 2)

        # grab the rotation matrix (applying the negative of the
        # angle to rotate clockwise), then grab the sine and cosine
        # (i.e., the rotation components of the matrix)
        M = cv2.getRotationMatrix2D((cX, cY), angle, 1.0)
        cos = np.abs(M[0, 0])
        sin = np.abs(M[0, 1])

        # compute the new bounding dimensions of the image
        nW = int((h * sin) + (w * cos))
        nH = int((h * cos) + (w * sin))

        # adjust the rotation matrix to take into account translation
        M[0, 2] += (nW / 2) - cX
        M[1, 2] += (nH / 2) - cY

        # perform the actual rotation and return the image
        return cv2.warpAffine(image, M, (nW, nH))

    def auto_canny(self, image, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(image)

        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)

        # return the edged image
        return edged

    def detect_object(self):
        self.template = self.auto_canny(self.template_img)

        #sizes of the original template
        (h, w) = self.template.shape[:2]

        # Create the rotated template images
        self.templates = [] # template images with list of elements [template, angle]
        self.template_angles = [] # rotation angles of the template images
        self.template_heights = []
        self.template_widths = []

        # Loop over angles
        for angle in np.linspace(self.min_angle, self.max_angle, self.num_angles):
            template_rotated = self.rotate_bound(self.template, angle)
            # template_rotated_edge = rotate_bound(template_edge, angle)
            
            (tH, tW) = template_rotated.shape[:2] # Sizes of the rotatsed template image
            self.templates.append(template_rotated) 
            self.template_angles.append(angle)
            self.template_widths.append(tW)
            self.template_heights.append(tH)

        # Find the max width and height (these will be the sizes of the all templates)
        self.h_max = max(self.template_heights)    
        self.w_max = max(self.template_widths) 
        self.tH, self.tW = self.h_max, self.w_max

        # Pad template images according so that all have the same size
        self.template_pixel_counts = []
        for i,img in enumerate(self.templates):
            delta_w = self.w_max - self.template_widths[i] 
            delta_h = self.h_max - self.template_heights[i]
            top, bottom = delta_h//2, delta_h-(delta_h//2)
            left, right = delta_w//2, delta_w-(delta_w//2)

            self.templates[i] = cv2.copyMakeBorder(img,top,bottom,left,right,cv2.BORDER_CONSTANT,None,[0,0,0])
            img_name = "debug_rotate_template_angle_{}_padded.png".format(self.template_angles[i])
            # cv2.imwrite(img_name, templates[i])
            print(img_name, self.templates[i].shape)
            # Save number of non-zero pixels in the rotated template to normalize later
            self.template_pixel_counts.append(cv2.countNonZero(self.templates[i]))


        found = None
        r = 1.0 # Size ratio between original template and the resized template (orginal/resized)

        gray = cv2.cvtColor(self.camera_img, cv2.COLOR_BGR2GRAY)
        # gray  = img

        # gray = cv2.GaussianBlur(gray,(5,5),0)
        # gray = img
        # Apply edge detection s
        # gray = cv2.Canny(gray, 50, 200)
        gray = self.auto_canny(gray)
        
        img_copy = self.camera_img.copy() # Copy original image to keep the original just in case

        # Loop over the rotated images
        for i,template in enumerate(self.templates):
            # Apply template Matching
            # res = cv2.matchTemplate(gray,template,self.method)
            # res = cv2.matchTemplate(gray,template,self.method, template_edge)
            res = cv2.matchTemplate(gray,template,self.method, template)
            
            (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(res)
            # Normalize min & max val.s
            # minVal = minVal * template_pixel_counts[i]
            # maxVal = maxVal * template_pixel_counts[i]
            
            # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
            if self.method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                # if we have found a new minimum correlation value, then update the bookkeeping variable
                if found is None or minVal < found[0]:
                    found = (minVal, minLoc, r, self.template_angles[i])
            else:
                # if we have found a new maximum correlation value, then update the bookkeeping variable
                if (found is None or maxVal > found[0]): 
                    found = (maxVal, maxLoc, r, self.template_angles[i])

        if found is not None:
            if self.method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                # unpack the bookkeeping variable and compute the (x, y) coordinates
                # of the bounding box based on the resized ratio
                (minVal, minLoc, r, angle) = found
                
                (startX, startY) = (int(minLoc[0]), int(minLoc[1]))
                (endX, endY) = (int((minLoc[0] + self.tW)), int((minLoc[1] + self.tH)))
                print("min: " + str(minVal) + ", r: " + str(r))
            else:
                # unpack the bookkeeping variable and compute the (x, y) coordinates
                # of the bounding box based on the resized ratio
                (maxVal, maxLoc, r, angle) = found

                (startX, startY) = (int(maxLoc[0]), int(maxLoc[1]))
                (endX, endY) = (int((maxLoc[0] + self.tW)), int((maxLoc[1] + self.tH)))
                print("max: " + str(maxVal) + ", r: " + str(r))
                
            (W, H) = (endX-startX ,endY-startY)
            P0 = (startX,startY)
            rr = RRect_center(P0,(W,H),(w,h),angle)
                
            print("angle: " + str(angle))
         
            # draw a bounding box around the detected result and display the image
            cv2.rectangle(img_copy, (startX, startY), (endX, endY), (0, 255, 0), 2)
            plt.imshow(img_copy)
            plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
            rr.draw(img_copy)

        cv2.imshow("Image", img_copy)
        cv2.waitKey(0) 






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
            matcher.detect_object()


            # return the pose of the object
            print("the object is found at the image coordinates:")
            print("<coordinates>")
            
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