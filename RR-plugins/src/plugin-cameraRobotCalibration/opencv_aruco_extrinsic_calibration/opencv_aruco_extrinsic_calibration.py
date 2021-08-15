import numpy as np
import cv2
import glob
import json

def rigid_transform_3D(A, B):
    # This function is taken from https://github.com/nghiaho12/rigid_transform_3D.git
    # Further information: http://nghiaho.com/?page_id=671
    
    # Input: expects 3xN matrix of points
    # Returns R,t
    # R = 3x3 rotation matrix
    # t = 3x1 column vector
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B

    return R, t

def calibrate(path_imgs, prefix, image_format, path_poses, saved_poses_filename, aruco_dict, aruco_id, aruco_markersize, mtx, dist):
    """ Apply extrinsic camera calibration operation for images in the given directory path 
    using opencv aruco marker detection, the extrinsic marker poses given in a json file, 
    and the given intrinsic camera parameters."""

    aruco_dict = eval(aruco_dict) # convert string to python
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict)
    aruco_params = cv2.aruco.DetectorParameters_create()


    # Create a dictionary for pose pairs
    # Structure of this disctionary is "filename":[[R_base2marker],[T_base2marker],[R_cam2marker],[T_cam2marker]]
    pose_pairs_dict = {}

    ## Read the poses file
    if path_poses[-1:] == '/':
        path_poses = path_poses[:-1]

    with open(path_poses+'/'+saved_poses_filename, 'r') as fp:
        saved_marker_poses_dict = json.load(fp) 
    # Note: Convert every R and T into numpy array in the dictionary (stored as list currently)

    # # debug
    # print(str(type(saved_marker_poses_dict)))
    # print(str(saved_marker_poses_dict))

    ## Read images one by one
    if path_imgs[-1:] == '/':
        path_imgs = path_imgs[:-1]

    if image_format[:1] == '.':
        image_format = image_format[1:]

    images = sorted(glob.glob(path_imgs+'/' + prefix + '*.' + image_format))

    print(path_imgs+'/' + prefix + '*.' + image_format)
    print(len(images))

    for fname in images:
        print(str(fname)) # debug
        img = cv2.imread(fname)
        
        # Find the aruco tag corners
        # corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params,cameraMatrix=mtx, distCoeff=dist)
        corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)

        # #debug
        # print(str(type(corners))) # <class 'list'>
        # print(str(corners))  # list of numpy arrays of corners
        # print(str(type(ids))) # <class 'numpy.ndarray'>
        # print(str(ids)) 

        if len(corners) > 0:
            # Only find the id that we desire
            indexes = np.flatnonzero(ids.flatten() == aruco_id).tolist()
            corners = [corners[index] for index in indexes]
            ids = np.asarray([ids[index] for index in indexes])

            # #debug
            # print(str(type(corners))) # <class 'list'>
            # print(str(corners))  # list of numpy arrays of corners
            # print(str(type(ids))) # <class 'numpy.ndarray'>
            # print(str(ids)) 

            if len(ids) > 0: # if there exist at least one id that we desire
                # Select the first detected one, discard the others
                corners = corners[0] # now corners is 1 by 4

                # # extract the marker corners (which are always returned
                # # in top-left, top-right, bottom-right, and bottom-left
                # # order)
                # corners = corners.reshape((4, 2))
                # (topLeft, topRight, bottomRight, bottomLeft) = corners

                # Estimate the pose of the detected marker in camera frame
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_markersize, mtx, dist)

                # # Debug: Show the detected tag and axis in the image
                # # # cv2.aruco.drawDetectedMarkers(img, corners)  # Draw A square around the markers (Does not work)
                # cv2.aruco.drawAxis(img, mtx, dist, rvec, tvec, aruco_markersize*0.75)  # Draw Axis
                # cv2.imshow(fname.split('/')[-1], img)
                # cv2.waitKey(250)

                # Convert tvec to one dimensional array, it was [[tvec]].
                T_cam2marker = tvec[0].flatten() # in camera's frame
                # Convert rvec to Rotation matrix
                R_cam2marker = cv2.Rodrigues(rvec[0])[0]

                # Store the calculated marker pose pair
                R_base2marker = np.asarray(saved_marker_poses_dict[fname.split('/')[-1]][0]) # Convert every R and T into numpy array in the dictionary
                T_base2marker = np.asarray(saved_marker_poses_dict[fname.split('/')[-1]][1])

                # Structure of this disctionary is "filename":[[R_base2marker],[T_base2marker],[R_cam2marker],[T_cam2marker]]
                pose_pairs_dict[fname.split('/')[-1]] = [R_base2marker,T_base2marker,R_cam2marker,T_cam2marker]
    
    # cv2.destroyAllWindows() # Debug
    print("HERE") # debug

    # Now its time to execute calibration
    src_lst = []
    dst_lst = []
    
    for filename, value in pose_pairs_dict.items():
        R_base2marker = value[0]
        T_base2marker = value[1]
        R_cam2marker = value[2]
        T_cam2marker = value[3]

        # print("------------------------")
        # print(filename)
        # # #debug
        # # print(str(type(R_base2marker)))
        # # print(str(R_base2marker))  
        # # print(str(type(T_base2marker))) 
        # # print(str(T_base2marker))
        # # #debug
        # # print(str(type(R_cam2marker)))
        # # print(str(R_cam2marker))  
        # # print(str(type(T_cam2marker))) 
        # # print(str(T_cam2marker)) 

        # # debug
        # print("My own calculation T_cam2base:")
        # print(str(T_cam2marker - (R_cam2marker @ R_base2marker.T @ T_base2marker) ))
        # print("My own calculation R_cam2base:")
        # print(str(R_cam2marker @ R_base2marker.T))

        # print("My own calculation T_base2cam:")
        # print(str(T_base2marker - (R_base2marker @ R_cam2marker.T @ T_cam2marker ) ))
        # print("My own calculation R_base2cam:")
        # print(str(R_base2marker @ R_cam2marker.T))
        # print("++++++++++++++++++++++++")

        src_lst.append(T_base2marker)
        dst_lst.append(T_cam2marker)
        
    # Convert lists to numpy arrays because this is what opencv wants
    src_arr = np.asarray(src_lst)
    dst_arr = np.asarray(dst_lst)

    # Finally execute the calibration
    R_cam2base,T_cam2base = rigid_transform_3D(src_arr.T,dst_arr.T)
    
    R_base2cam = R_cam2base.T
    T_base2cam = - R_base2cam @ T_cam2base
    
    #debug
    print("FINAL RESULTS: ")
    print("str(R_cam2base)")
    # print(str(type(R_cam2base)))
    print(str(R_cam2base))
    print("str(T_cam2base)")
    # print(str(type(T_cam2base.flatten())))
    print(str(T_cam2base))
    
    print("str(R_base2cam)")
    # print(str(type(R_base2cam)))
    print(str(R_base2cam))
    print("str(T_base2cam)")
    # print(str(type(T_base2cam.flatten())))
    print(str(T_base2cam))
    
    return [R_cam2base,T_cam2base.flatten(),R_base2cam,T_base2cam.flatten()]