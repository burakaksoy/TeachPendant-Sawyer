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

     
    c=RRN.ConnectService(url)

    p=c.frame_stream_compressed.Connect(-1)

    #Set the callback for when a new pipe packet is received to the
    #new_frame function
    p.PacketReceivedEvent+=new_frame
    try:
        c.start_streaming()
    except: pass

    cv2.namedWindow("Image")

    while True:
        #Just loop resetting the frame
        #This is not ideal but good enough for demonstration

        if (not current_frame is None):
            cv2.imshow("Image",current_frame)
        if cv2.waitKey(10)!=-1:
            break
    cv2.destroyAllWindows()

    p.Close()
    # c.StopStreaming()
    c.stop_streaming()


    
current_frame = None

def new_frame(pipe_ep):
    global current_frame

    #Loop to get the newest frame
    while (pipe_ep.Available > 0):
        #Receive the packet
        image=pipe_ep.ReceivePacket()
        #Convert the packet to an image and set the global variable
        print(len(image.data))
        current_frame=cv2.imdecode(image.data,1)


if __name__ == '__main__':
    main()