
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 22 04:35:53 2021

@author: burak
TO TEST ROTATED RECTANGULAR ROI CROP
"""

import shapely
import shapely.geometry
import cv2
import numpy as np

# Load test image
camera_img = cv2.imread("test.jpg")
(h, w) = camera_img.shape[:2]
(cX, cY) = (w / 2, h / 2)

# ROI parameters
ROI_x = 640
ROI_y = 140
ROI_w = 420
ROI_h = 310
ROI_theta = 45.0 # degree

(h, w) = camera_img.shape[:2]
geom_camera_img = shapely.geometry.box(0.0,-h,w,0.0,ccw=True)

# define ROI rectangle geometrically
geom_ROI = shapely.geometry.box(-ROI_w/2.,-ROI_h/2.,ROI_w/2.,ROI_h/2.,ccw=True)
# translate the ROI to its defined centroid position in image frame
geom_ROI = shapely.affinity.translate(geom_ROI, xoff = ROI_x, yoff = -ROI_y)
# rotate the ROI as defined around its centroid CCW
geom_ROI = shapely.affinity.rotate(geom_ROI, ROI_theta, origin='centroid', use_radians=False)


# define ROI and camera_img intersection geometrically
geom_intersection = geom_camera_img.intersection(geom_ROI)
print(geom_intersection )

## debug: visualize the defined boxes
#from matplotlib import pyplot
#from descartes import PolygonPatch
#from shapely.figures import SIZE, BLUE, GRAY, set_limits
#
#fig = pyplot.figure(1, figsize=SIZE, dpi=90)
#ax = fig.add_subplot(121)
#
#patch1 = PolygonPatch(geom_camera_img, fc=GRAY, ec=GRAY, alpha=0.2, zorder=1)
#ax.add_patch(patch1)
#patch2 = PolygonPatch(geom_ROI, fc=GRAY, ec=GRAY, alpha=0.2, zorder=1)
#ax.add_patch(patch2)
#
#patchc = PolygonPatch(geom_intersection, fc=BLUE, ec=BLUE, alpha=0.5, zorder=2)
#ax.add_patch(patchc)
#
#ax.set_title('a.intersection(b)')
#
#set_limits(ax, -2*(w+1), 2*(w+1), -2*(h+1), 2*(h+1))
#
#pyplot.show()

# export intersection polygon coodinates as our final ROI
ROI_coords = np.array(geom_intersection.exterior.coords,dtype=np.int32)
# Convert polygon coordinates into image frame
ROI_coords[:,1] = -ROI_coords[:,1]

# Create a mask for camera_img
ROI_mask = np.zeros(camera_img.shape, dtype=np.uint8)

channel_count = camera_img.shape[2]  # i.e. 3 or 4 depending on your image
ignore_mask_color = (255,)*channel_count
cv2.fillConvexPoly(ROI_mask, ROI_coords, ignore_mask_color)
# cv2.fillConvexPoly if you know it's convex, otherwise use cv2.fillPoly

# apply the mask
masked_image = cv2.bitwise_and(camera_img, ROI_mask)

# save the result
cv2.imwrite('image_masked.png', masked_image)

# Find ROI bounding box as (minx, miny, maxx, maxy)
ROI_bbox = np.array(geom_intersection.bounds, dtype=np.int32)

# define ROI bounding box rectangle geometrically
geom_ROI_bbox = shapely.geometry.box(ROI_bbox[0],ROI_bbox[1],ROI_bbox[2],ROI_bbox[3],ccw=True)

# Convert bounding box coordinates into image frame
ROI_bbox[[1,3]] = -ROI_bbox[[3,1]]

# Lets crop the intersection are from the image
masked_cropped_image = masked_image[ROI_bbox[1]:ROI_bbox[3],ROI_bbox[0]:ROI_bbox[2]]

# save the result
cv2.imwrite('masked_cropped_image.png', masked_cropped_image)


