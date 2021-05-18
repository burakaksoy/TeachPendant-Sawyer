"""
Created on Thu Apr 29 21:58:01 2021

@author: burak
TO TEST 3D POINT RECONSTRUCTION GIVEN A 2D IMAGE POINT
"""

import cv2
import numpy as np

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

img_filename = "../calibration_imgs_backup/images0.png"

# Load image
img = cv2.imread(img_filename) 

# Show the template image
cv2.imshow("img",img)
print("press any key to continue..")
cv2.waitKey(0)
cv2.destroyAllWindows()

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

width = 7
height = 6

# Find the chess board corners
ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

# If found, add object points, image points (after refining them)
if ret:
    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    # Draw and display the corners
    img2 = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
    cv2.imshow("corners", img2)
    print("press any key to continue..")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
# %% Load camera parameters and origin pose wrt camera
cv_file = cv2.FileStorage("../calibration_files/camera1.yml", cv2.FILE_STORAGE_READ)

# note we also have to specify the type to retrieve other wise we only get a
# FileNode object back instead of a matrix
mtx = cv_file.getNode("K").mat()
dist = cv_file.getNode("D").mat()
R_co = cv_file.getNode("R_co").mat()
tvec = cv_file.getNode("T_co").mat()
value_z_distance = tvec[2,0] # ~730 mm

cv_file.release()

# %% Now, the corners2[0] in 2D should correspond to the point tvec in 3D
# let's reconstruct corners2[0]
corner = corners2[0][0]
x = corner[0]
y = corner[1]
theta = 0.0
src = np.asarray([x,y], dtype=np.float32)
src = np.reshape(src,(-1,1,2)) # Rehsape as opencv requires (N,1,2)

# Find the corresponding world pose of the detected pose in camera frame
dst = cv2.undistortPoints(src,mtx,dist) # dst is Xc/Zc and Yc/Zc in the same shape of src
dst = dst * float(value_z_distance)
dst = np.squeeze(dst) 

# Finally the translation between the detected object center and the camera frame represented in camera frame is T = [Xc,Yc,Zc]
Xc = dst[0]
Yc = dst[1]
Zc = float(value_z_distance)
T = np.asarray([Xc,Yc,Zc])

print("tvec(original 3D vector):")
print(np.squeeze(tvec))
print("T   (reconstructed 3D vector):")
print(T)

#%% Let's do another reconstruction using the point at the farthest corner from the origin
square_size = 29.0 # mm
T_far_corner = np.asarray([[(width-1)*square_size],[(height-1)*square_size],[0.]])
# Transform this point into camera frame
T_far_corner = tvec + R_co @ T_far_corner
value_z_distance = T_far_corner[2,0] # ~704 mm
#value_z_distance = 730.27569446

# The farthest corner is the last elemen of the corners, ie corners2[-1]
corner = corners2[-1][0]
x = corner[0]
y = corner[1]
theta = 0.0
src = np.asarray([x,y], dtype=np.float32)
src = np.reshape(src,(-1,1,2)) # Rehsape as opencv requires (N,1,2)

# Find the corresponding world pose of the detected pose in camera frame
dst = cv2.undistortPoints(src,mtx,dist) # dst is Xc/Zc and Yc/Zc in the same shape of src
dst = dst * float(value_z_distance)
dst = np.squeeze(dst) 

# Finally the translation between the detected object center and the camera frame represented in camera frame is T = [Xc,Yc,Zc]
Xc = dst[0]
Yc = dst[1]
Zc = float(value_z_distance)
T = np.asarray([Xc,Yc,Zc])

print("T_far_corner(original 3D vector):")
print(np.squeeze(T_far_corner))
print("T   (reconstructed 3D vector):")
print(T)