#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 25 00:18:08 2021

@author: burak

TO TEST TEMPLATE MATCHING WITH ROTATED RECTANGULAR ROI CROP
"""

import cv2
import numpy as np

from opencv_template_matching_ROI import TemplateMatchingMultiAngleWithROI


#obj_img_filenames = ["test_template0.png","test_template1.png","test_template2.png","test_template3.png"]
obj_img_filenames = ["template_match_test_template_image_real2.png"]
#obj_img_filenames = ["template_match_test_template_image.png"]

#compressed_cam_img_filenames = ["test_objects0.png","test_objects1.png","test_objects2.png","test_objects3.png","test_objects4.png"] # Camera image
compressed_cam_img_filenames = ["template_match_test_image_real.png"] # Camera image
#compressed_cam_img_filenames = ["template_match_test_image.png"] # Camera image

# deifne # ROI parameters
#ROI_x = 600
#ROI_y = 300 
#ROI_w = 700
#ROI_h = 700
#ROI_theta = 0.0 # degree

ROI_x = 460
ROI_y = 500 # try 100 for upper perfume
ROI_w = 320
ROI_h = 300
ROI_theta = 0.0 # degree
            
#ROI_x = 950
#ROI_y = 680
#ROI_w = 300
#ROI_h = 200
#ROI_theta = 0.0 # degree

ROI_parameters_list = [(ROI_x,ROI_y,ROI_w,ROI_h,ROI_theta),None]

for obj_img_filename in obj_img_filenames:
    for compressed_cam_img_filename in compressed_cam_img_filenames:
        for ROI_parameters in ROI_parameters_list:
            print (obj_img_filename)
            print (compressed_cam_img_filename)
            print (ROI_parameters)
            
            
            
            #obj_img_filename = "template_match_test_template_image.png" # Template image
            #obj_img_filename = "template_match_test_template_image_real.png" # Template image
            #obj_img_filename = "test_template3.png" # Template image
            
            
            #compressed_cam_img_filename = "template_match_test_image.png" # Camera image
            #compressed_cam_img_filename = "template_match_test_image_real.png" # Camera image
            #compressed_cam_img_filename = "test_objects1.png" # Camera image
            
            return_result_image = True
            
            # Load test image
            img_obj = cv2.imread(obj_img_filename)             
            
            #img_obj = cv2.cvtColor(img_obj, cv2.COLOR_BGR2GRAY)
            #img_obj = cv2.equalizeHist(img_obj)
            
            # Show the template image
            #cv2.imshow("template image",img_obj)
            #print("press any key to continue template matching")
            #cv2.waitKey(0)
            
            #Load camera image
            img_compressed_cam = cv2.imread(compressed_cam_img_filename)
            
            #img_compressed_cam = cv2.cvtColor(img_compressed_cam, cv2.COLOR_BGR2GRAY)
            #img_compressed_cam = cv2.equalizeHist(img_compressed_cam)
            
            # Show the camera image
            #cv2.imshow("camera image",img_compressed_cam)
            #print("press any key to continue template matching")
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
            
            
            
            #ROI_parameters = (ROI_x,ROI_y,ROI_w,ROI_h,ROI_theta)
            
            # If you want to make ROI the whole camera image
            #ROI_parameters = None
            
            min_ROI_intersection_area = 20.0 # Default is 20.0
            
            matcher = TemplateMatchingMultiAngleWithROI(img_obj,img_compressed_cam,ROI_parameters,min_ROI_intersection_area)
            
            if return_result_image:
                center, wh, angle, detection_result_img = matcher.detect_object(return_result_image)
            else:
                center, wh, angle = matcher.detect_object()
                detection_result_img = None
                
            # return the pose of the object
#            print("the object is found..:")
#            print("center coordinates in img frame(x,y): " + str(center))
#            print("(w,h): " + str(wh))
#            print("angle: " + str(angle))
            
            if return_result_image:
                cv2.imshow("detection results",detection_result_img)
                print("press any key to finish template matching")
                cv2.waitKey(0)
            cv2.destroyAllWindows()
