import numpy as np
import cv2
import math
# from matplotlib import pyplot as plt

import shapely
import shapely.geometry


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

class TemplateMatchingMultiAngleWithROI(object):
    def __init__(self, template_img, camera_img, ROI_parameters = None, min_ROI_intersection_area = 20.0):
        # INPUTs:
        #   template_img: image of the object that we try to find in the camera_img
        #   camera_img: image that we trying to find the object in it
        #   ROI_parameters: A tuple that defines ROI in the image for template matching to be done (x,y,w,h,theta)
        #       x,y: center coordinates of the ROI (float) in image coordinates
        #       w,h: width and height of the ROI (float)
        #       theta: CCW rotation angle of the ROI (degree, float)
        #   min_ROI_intersection_area: minimum intersection area btw. ROI and the image for ROI to be acceptible.

        self.template_img = template_img
        self.camera_img = camera_img

        # Define ROI related parameters
        if ROI_parameters == None: 
            # camera_img itself is the region of interest
            (h, w) = self.camera_img.shape[:2]
            (cX, cY) = (w / 2, h / 2)
            self.ROI_x = cX
            self.ROI_y = cY
            self.ROI_w = w
            self.ROI_h = h
            self.ROI_theta = 0.0
        else:
            # ROI is defined by the user
            self.ROI_x = ROI_parameters[0]
            self.ROI_y = ROI_parameters[1]
            self.ROI_w = ROI_parameters[2]
            self.ROI_h = ROI_parameters[3]
            self.ROI_theta = ROI_parameters[4]

        # define camera_img rectangle geometrically in image frame
        # shapely.geometry.box(minx, miny, maxx, maxy, ccw=True)
        (h, w) = self.camera_img.shape[:2]
        self.geom_camera_img = shapely.geometry.box(0.0,-h,w,0.0,ccw=True)
        
        # define ROI rectangle geometrically
        self.geom_ROI = shapely.geometry.box(-self.ROI_w/2.,-self.ROI_h/2.,self.ROI_w/2.,self.ROI_h/2.,ccw=True)
        # translate the ROI to its defined centroid position in image frame
        self.geom_ROI = shapely.affinity.translate(self.geom_ROI, xoff = self.ROI_x, yoff = -self.ROI_y)
        # rotate the ROI as defined around its centroid CCW
        self.geom_ROI = shapely.affinity.rotate(self.geom_ROI, self.ROI_theta, origin='centroid', use_radians=False)

        # define ROI and camera_img intersection geometrically
        self.geom_intersection = self.geom_camera_img.intersection(self.geom_ROI)
        
        # Assert that the defined ROI is acceptible (i.e the min. intersection area within the ROI and camera_img is more than a threshold value)
        self.min_intersection_area = min_ROI_intersection_area
        
        assert self.geom_intersection.area >= self.min_intersection_area, "ROI intersection with Image is not large enough!"

        # export intersection polygon coodinates as our final ROI
        self.ROI_coords = np.array(self.geom_intersection.exterior.coords,dtype=np.int32)
        # Convert polygon coordinates into image frame
        self.ROI_coords[:,1] = -self.ROI_coords[:,1]

        # Create a mask for camera_img
        self.ROI_mask = np.zeros(self.camera_img.shape, dtype=np.uint8)

        channel_count = self.camera_img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,)*channel_count
        cv2.fillConvexPoly(self.ROI_mask, self.ROI_coords, ignore_mask_color)
        # cv2.fillConvexPoly if you know it's convex, otherwise use cv2.fillPoly

        # apply the mask
        self.masked_image = cv2.bitwise_and(self.camera_img, self.ROI_mask)

        # # save the result
        # cv2.imwrite('image_masked.png', self.masked_image)

        # Find ROI bounding box as (minx, miny, maxx, maxy)
        self.ROI_bbox = np.array(self.geom_intersection.bounds, dtype=np.int32)

        # define ROI bounding box rectangle geometrically
        self.geom_ROI_bbox = shapely.geometry.box(self.ROI_bbox[0],self.ROI_bbox[1],self.ROI_bbox[2],self.ROI_bbox[3],ccw=True)

        # Convert bounding box coordinates into image frame
        self.ROI_bbox[[1,3]] = -self.ROI_bbox[[3,1]]

        # Lets crop the intersection are from the image
        self.masked_cropped_image = self.masked_image[self.ROI_bbox[1]:self.ROI_bbox[3],self.ROI_bbox[0]:self.ROI_bbox[2]]

        # # save the result
        # cv2.imwrite('masked_cropped_image.png', self.masked_cropped_image)

        self.ROI_bbox_w = self.ROI_bbox[2]-self.ROI_bbox[0]
        self.ROI_bbox_h = self.ROI_bbox[3]-self.ROI_bbox[1]

        # Define template matching parameters
        self.max_angle = 180.0 # degree
        self.min_angle = -180.0 # degree
        self.num_angles = 19 # number of different angles (has to be integer)

        # There are 6 Template Matching methods 
        # self.method = cv2.TM_CCOEFF # good
        self.method = cv2.TM_CCOEFF_NORMED  # best
        # self.method = cv2.TM_CCORR # not good # Seems to be best w/out canny
        # self.method = cv2.TM_CCORR_NORMED # second best
        # self.method = cv2.TM_SQDIFF # terrible
        # self.method = cv2.TM_SQDIFF_NORMED # terrible
    
        self.show_visuals = False

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

    def detect_object(self, return_result_image = False):
        self.template = self.auto_canny(self.template_img)

        if self.show_visuals:
            # Show template to the user 
            # cv2.imshow("Template", np.array(template, dtype = np.uint8 ))
            cv2.imshow("Template", template)
            cv2.waitKey(0)

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

        # Assert that the template images are smaller than the masked cropped image
        assert self.ROI_bbox_w >= self.tW, "Template Image width should be less than width of searched image ROI!"
        assert self.ROI_bbox_h >= self.tH, "Template Image height should be less than height of searched image ROI!"

        # Pad template images according so that all have the same size
        # self.template_pixel_counts = []
        for i,img in enumerate(self.templates):
            delta_w = self.w_max - self.template_widths[i] 
            delta_h = self.h_max - self.template_heights[i]
            top, bottom = delta_h//2, delta_h-(delta_h//2)
            left, right = delta_w//2, delta_w-(delta_w//2)

            self.templates[i] = cv2.copyMakeBorder(img,top,bottom,left,right,cv2.BORDER_CONSTANT,None,[0,0,0])
            img_name = "debug_rotate_template_angle_{}_padded.png".format(self.template_angles[i])
            # cv2.imwrite(img_name, templates[i])
            # print(img_name, self.templates[i].shape)
            # # Save number of non-zero pixels in the rotated template to normalize later
            # self.template_pixel_counts.append(cv2.countNonZero(self.templates[i]))


        found = None
        r = 1.0 # Size ratio between original template and the resized template (orginal/resized)

        # gray = cv2.cvtColor(self.camera_img, cv2.COLOR_BGR2GRAY) # This may not be necessary since opencv can handle rgb images directly for canny edge detection today (09jan2021) But converting to gray helps for faster template matching so keep it 
        gray = cv2.cvtColor(self.masked_cropped_image, cv2.COLOR_BGR2GRAY) # This may not be necessary since opencv can handle rgb images directly for canny edge detection today (09jan2021) But converting to gray helps for faster template matching so keep it 
        # gray = cv2.equalizeHist(gray) # You can comment out this, not really improves accuracy
        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        # gray = clahe.apply(gray)

        # gray  = img

        # gray = cv2.GaussianBlur(gray,(5,5),0)
        # gray = img
        # Apply edge detection s
        # gray = cv2.Canny(gray, 50, 200)
        gray = self.auto_canny(gray)
        
        if self.show_visuals or return_result_image:
            # img_copy = self.camera_img.copy() # Copy original image to keep the original just in case
            img_copy = self.masked_cropped_image.copy() # Copy original image to keep the original just in case
            

        # Loop over the rotated images
        for i,template in enumerate(self.templates):
            # Apply template Matching
            # res = cv2.matchTemplate(gray,template,self.method)
            # res = cv2.matchTemplate(gray,template,self.method, template_edge)
            res = cv2.matchTemplate(gray,template,self.method, template)
            
            (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(res)
            # Normalize min & max val.s
            # minVal = minVal * self.template_pixel_counts[i]
            # maxVal = maxVal * self.template_pixel_counts[i]
            
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
                 
            (W, H) = (endX-startX ,endY-startY) # Outer ractangle widht and height
            P0 = (startX,startY) # The first corner coordinates in img frame of the outer rectangle

            if self.show_visuals or return_result_image:
                rr = RRect_center(P0,(W,H),(w,h),angle)     
                # draw a bounding box around the detected result and display the image
                cv2.rectangle(img_copy, (startX, startY), (endX, endY), (0, 255, 0), 2)
                # plt.imshow(img_copy)
                # plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
                rr.draw(img_copy)

        if self.show_visuals:
            cv2.imshow("Image", img_copy)
            cv2.waitKey(0)

        
        # return center coordinates of the detected object;  w,h of the detected object in image; orientation angle in degrees
        # self.center = (int(P0[0]+W/2.0),int(P0[1]+H/2.0)) # center point coordinates
        self.center = (int(P0[0]+W/2.0+self.ROI_bbox[0]),int(P0[1]+H/2.0+self.ROI_bbox[1])) # center point coordinates in original image frame!

        self.wh = (w,h) # widht and height of the detected object in image
        self.angle = angle # orientation angle in degrees
        
        # print("center coordinates in img frame: " + str(self.center))
        # print("(w,h): " + str(self.wh))
        # print("angle: " + str(self.angle))

        if return_result_image:
            return self.center, self.wh, self.angle, img_copy
        else:
            return self.center, self.wh, self.angle