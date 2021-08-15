import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

import cv2
import threading

class CameraFeedback_impl(object):
    def __init__(self):
        self.url_camera = None 
        self.camera_sub = None
        self.camera = None # RR camera object

        # Define feedback image sizes, the camera images will be resized to these values
        self.output_W = 320
        self.output_H = 240

        self._image_consts = RRN.GetConstants('com.robotraconteur.image')
        self._image_type = RRN.GetStructureType('com.robotraconteur.image.Image')
        self._image_info_type = RRN.GetStructureType('com.robotraconteur.image.ImageInfo')
        self._compressed_image_type = RRN.GetStructureType('com.robotraconteur.image.CompressedImage')

        self._capture_lock = threading.Lock()
        self._streaming = False

    def RRServiceObjectInit(self, ctx, service_path):
        self._downsampler = RR.BroadcastDownsampler(ctx)
        self._downsampler.AddPipeBroadcaster(self.preview_stream_out)
        self.preview_stream_out.MaxBacklog = 2

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

    def frame_threadfunc(self):
        while(self._streaming):
            with self._capture_lock:
                
                mat = self.current_frame # Like reading a frame from camera
                if mat is None:
                    #TODO: notify user?
                    # self._streaming=False
                    # print("no frame is arrived")
                    continue
            self.preview_stream_out.AsyncSendPacket(self._cv_mat_to_compressed_image(mat),lambda: None)
            # print("Streamed")

    def start_streaming_out(self):
        if (self._streaming):
            raise RR.InvalidOperationException("Already streaming")
        self._streaming=True
        t=threading.Thread(target=self.frame_threadfunc)
        t.start()
        print("Streamed_staaart")

    def stop_streaming_out(self):
        if (not self._streaming):
            raise RR.InvalidOperationException("Not streamingg")
        self._streaming=False
        print("Streamed_stooopp")


    def reset(self):
        self.stop_streaming_out()
        self.stop_stream_from_cam()

        self.url_camera = None 
        self.camera_sub = None
        self.camera = None # RR camera object
        
    def connect2camera(self, url_camera):
        if self.camera is None:
            self.url_camera = url_camera

            self.camera = RRN.ConnectService(self.url_camera) # connect to robot with the given url
            # self.camera_sub = RRN.SubscriberService(self.url_camera)
            # self.camera = self.camera_sub.GetDefaultClientWait(1)

            # Define camera modes

            self.assign_camera_details()

            self.start_stream_from_cam()

            self.start_streaming_out()

            # log that the camera is successfully connected
            print("Camera is connected to CameraFeedback service!")
        else:
            # Give an error that says the camera is already connected
            print("Camera is already connected to CameraFeedback service! Trying to connect again..")
            self.reset()
            self.connect2camera(url_camera)        

    def assign_camera_details(self):
        if self.camera is not None:
            # TODO: Later it can be used for storing camera parameters etc
            pass
        else:
            # Give an error message to show that the robot is not connected
            print("Assign camera details failed. camera is not connected to CameraFeedback service yet!")

    
    def stop_stream_from_cam(self):
        self.pipe_preview_stream.Close()
        try:
            self.camera.stop_streaming()
        except:
            pass
        RRN.DisconnectService(self.camera)

    def start_stream_from_cam(self):
        self.current_frame = None 
        
        self.pipe_preview_stream = self.camera.preview_stream.Connect(-1)

        #Set the callback for when a new pipe packet is received to the
        #new_frame function
        self.pipe_preview_stream.PacketReceivedEvent += self.new_frame

        try:
            self.camera.start_streaming()
            print("Streaming input started from connected camera")
        except: 
            pass


    def new_frame(self, pipe_ep):
        #Loop to get the newest frame
        while (pipe_ep.Available > 0):
            #Receive the packet
            image_in = pipe_ep.ReceivePacket() # incoming RR image
            #Convert the packet to an image 
            current_frame = cv2.imdecode(image_in.data,1) # This is a cv_mat

            # prepare_incoming_img_for_output
            if not(current_frame is None):
                # Resize the image for output size, choose whichever is smaller
                org_w = image_in.image_info.width
                org_h = image_in.image_info.height
                # print(org_w ,org_h)

                # if the incoming image is bigger than the desired image, scale down.
                if org_w > self.output_W  or org_h > self.output_H:
                    scale_w = float(self.output_W)/org_w
                    scale_h = float(self.output_H)/org_h
                    scale = min(scale_h,scale_w)

                    width = int(org_w * scale)
                    height = int(org_h * scale)
                    dim = (width, height)
                    self.current_frame = cv2.resize(current_frame, dim, interpolation = cv2.INTER_AREA)
                else:
                    self.current_frame = current_frame

def main():
    rr_args = ["--robotraconteur-jumbo-message=true"] + sys.argv

    # register service type
    # RRN.RegisterServiceTypeFromFile("./experimental.pluginCameraFeedback")
    RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.image',"./experimental.pluginCameraFeedback"],True)

    # create object
    CameraFeedback_inst = CameraFeedback_impl()

    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-cameraFeedback-service", 8889,argv=rr_args):
        
        # register service with service name "CameraFeedback", type "experimental.pluginCameraFeedback.CameraFeedback", actual object: CameraFeedback_inst
        service_ctx = RRN.RegisterService("CameraFeedback","experimental.pluginCameraFeedback.CameraFeedback",CameraFeedback_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginCameraFeedback Server started, press enter to quit...")
        else:
            raw_input("pluginCameraFeedback Server started, press enter to quit...")

if __name__ == '__main__':
    main()