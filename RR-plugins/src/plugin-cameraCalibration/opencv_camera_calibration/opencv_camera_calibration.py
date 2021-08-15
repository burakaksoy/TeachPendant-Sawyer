import numpy as np
import cv2
import glob
import argparse

def calibrate(dirpath, prefix, image_format, square_size, width=9, height=6):
    """ Apply camera calibration operation for images in the given directory path. """

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
    objp = np.zeros((height*width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

    objp = objp * square_size

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    if dirpath[-1:] == '/':
        dirpath = dirpath[:-1]

    if image_format[:1] == '.':
        image_format = image_format[1:]

    images = sorted(glob.glob(dirpath+'/' + prefix + '*.' + image_format))

    print(dirpath+'/' + prefix + '*.' + image_format)
    print(len(images))

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # # Draw and display the corners
            # img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
            # cv2.imshow(fname, img)
            # cv2.waitKey(1000)
    
    # cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    # print(str(dist))
    # print("mtx")
    # print(str(mtx))
    
    # img = cv2.imread(images[0])
    # h,  w = img.shape[:2]
    # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    # print("newcameramtx")
    # print(str(newcameramtx))
    
    # # undistort
    # dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    # # # crop the image
    # # x, y, w, h = roi
    # # dst = dst[y:y+h, x:x+w]
    # cv2.imwrite('calibresult_newcameramtx.png', dst)
    
    # dst = cv2.undistort(img, mtx, dist, None, mtx)
    # cv2.imwrite('calibresult_mtx.png', dst)

    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )

    return [ret, mtx, dist, rvecs, tvecs]

    
def save_coefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    # Camera Matrix
    cv_file.write("K", mtx)
    # Distortion Coefficients
    cv_file.write("D", dist)
    # Rotation matrix 
    R_co, _ = cv2.Rodrigues(rvecs[0]) 
    cv_file.write("R_co", R_co )
    # Tranlastion vector
    cv_file.write("T_co", tvecs[0])
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

def load_coefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("K").mat()
    dist_matrix = cv_file.getNode("D").mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]
    
    
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Camera calibration')
    parser.add_argument('--image_dir', type=str, required=True, help='image directory path')
    parser.add_argument('--image_format', type=str, required=True,  help='image format, png/jpg')
    parser.add_argument('--prefix', type=str, required=True, help='image prefix')
    parser.add_argument('--square_size', type=float, required=False, help='chessboard square size')
    parser.add_argument('--width', type=int, required=False, help='chessboard width size, default is 9')
    parser.add_argument('--height', type=int, required=False, help='chessboard height size, default is 6')
    parser.add_argument('--save_file', type=str, required=True, help='YML file to save calibration matrices')

    args = parser.parse_args()
    ret, mtx, dist, rvecs, tvecs = calibrate(args.image_dir, args.prefix, args.image_format, args.square_size, args.width, args.height)
    save_coefficients(mtx, dist, args.save_file)
    print("Calibration is finished. RMS: ", ret)
