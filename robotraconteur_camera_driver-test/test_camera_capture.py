from RobotRaconteur.Client import *
import time
import numpy
import cv2
import sys

def WebcamImageToMat(image):
    frame2=image.data.reshape([image.image_info.height, image.image_info.width, 3], order='C')
    return frame2

def main():

    url='rr+tcp://localhost:59823/?service=camera'
    #if (len(sys.argv)>=2):
    #    url=sys.argv[1]

     
    c1=RRN.ConnectService(url)
    
    frame1=WebcamImageToMat(c1.capture_frame())
    
    #cv2.imshow("camera",frame1)
    
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    while True:
        compressed_image = c1.capture_frame_compressed()
        frame2 = cv2.imdecode(compressed_image.data,1)
        
        cv2.imshow("camera",frame2)
        cv2.waitKey(1)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()