import cv2
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import argparse
import sys
import platform
import threading
import numpy as np




class CameraImpl(object):
    
    def __init__(self, device_id, width, height, fps):
        
        #if platform.system() == "Windows":
        #    self._capture = cv2.VideoCapture(device_id + cv2.CAP_DSHOW)
        #else:
        self._capture = cv2.VideoCapture(device_id)
        #assert self._capture.isOpened(), f"Could not open device: {device_id}"
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._capture.set(cv2.CAP_PROP_FPS, fps)

        self._imaging_consts = RRN.GetConstants('com.robotraconteur.imaging')
        self._image_consts = RRN.GetConstants('com.robotraconteur.image')
        self._image_type = RRN.GetStructureType('com.robotraconteur.image.Image')
        self._image_info_type = RRN.GetStructureType('com.robotraconteur.image.ImageInfo')
        self._compressed_image_type = RRN.GetStructureType('com.robotraconteur.image.CompressedImage')
        self._date_time_utc_type = RRN.GetPodDType('com.robotraconteur.datetime.DateTimeUTC')
        self._isoch_info = RRN.GetStructureType('com.robotraconteur.device.isoch.IsochInfo')
        self._capture_lock = threading.Lock()
        self._streaming = False
        self._fps = self._capture.get(cv2.CAP_PROP_FPS)

    def RRServiceObjectInit(self, ctx, service_path):
        self._downsampler = RR.BroadcastDownsampler(ctx)
        self._downsampler.AddPipeBroadcaster(self.frame_stream)
        self._downsampler.AddPipeBroadcaster(self.frame_stream_compressed)
        self._downsampler.AddPipeBroadcaster(self.preview_stream)
        self.frame_stream.MaxBacklog = 2
        self.frame_stream_compressed.MaxBacklog = 2
        self.preview_stream.MaxBacklog = 2

    def _cv_mat_to_image(self, mat):

        is_mono = False
        if (len(mat.shape) == 2 or mat.shape[2] == 1):
            is_mono = True

        image_info = self._image_info_type()
        image_info.width =mat.shape[1]
        image_info.height = mat.shape[0]
        if is_mono:
            image_info.step = mat.shape[1]
            image_info.encoding = self._image_consts["ImageEncoding"]["mono8"]
        else:
            image_info.step = mat.shape[1]*3
            image_info.encoding = self._image_consts["ImageEncoding"]["rgb8"]

        image = self._image_type()
        image.image_info = image_info
        image.data=mat.reshape(mat.size, order='C')
        return image

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

    def capture_frame(self):
        with self._capture_lock:
            ret, mat=self._capture.read()
            if not ret:
                raise RRN.OperationFailedException("Could not read from camera")
        return self._cv_mat_to_image(mat)

    def capture_frame_compressed(self):
        with self._capture_lock:
            ret, mat=self._capture.read()
            if not ret:
                raise RRN.OperationFailedException("Could not read from camera")
        return self._cv_mat_to_compressed_image(mat)

    def trigger():
        raise RR.NotImplementedException("Not available on this device")

    def frame_threadfunc(self):
        while(self._streaming):
            with self._capture_lock:
                ret, mat=self._capture.read()
                if not ret:
                    #TODO: notify user?
                    self._streaming=False
                    continue
            
            self.frame_stream.AsyncSendPacket(self._cv_mat_to_image(mat),lambda: None)
            self.frame_stream_compressed.AsyncSendPacket(self._cv_mat_to_compressed_image(mat),lambda: None)
            self.preview_stream.AsyncSendPacket(self._cv_mat_to_compressed_image(mat,70),lambda: None)

    def start_streaming(self):
        if (self._streaming):
            raise RR.InvalidOperationException("Already streaming")
        self._streaming=True
        t=threading.Thread(target=self.frame_threadfunc)
        t.start()

    def stop_streaming(self):
        if (not self._streaming):
            raise RR.InvalidOperationException("Not streaming")
        self._streaming=False

    @property
    def isoch_downsample(self):
        return self._downsampler.GetClientDownsample(RR.ServerEndpoint.GetCurrentEndpoint())

    @isoch_downsample.setter
    def isoch_downsample(self, value):
        return self._downsampler.SetClientDownsample(RR.ServerEndpoint.GetCurrentEndpoint(),value)

    @property
    def isoch_info(self):
        ret = self._isoch_info()
        ret.update_rate = self._fps
        ret.max_downsample = 100
        ret.isoch_epoch = np.zeros((1,),dtype=self._date_time_utc_type)

    @property
    def capabilities(self):
        return 0x1 | 0x2 | 0x4

    

def main():
    parser = argparse.ArgumentParser(description="OpenCV based camera driver service for Robot Raconteur")
    parser.add_argument("--device-id", type=int, default=0, help="the device to open (default 0)")
    parser.add_argument("--width", type=int, default=1280, help="try to set width of image (default 1280)")
    parser.add_argument("--height", type=int, default=720, help="try to set height of image (default 720)")
    parser.add_argument("--fps", type=int, default=15, help="try to set rate of video capture (default 15 fps)")
    parser.add_argument("--wait-signal",action='store_const',const=True,default=False, help="wait for SIGTERM orSIGINT (Linux only)")

    args, _ = parser.parse_known_args()

    rr_args = ["--robotraconteur-jumbo-message=true"] + sys.argv

    RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.imaging'],True)

    camera = CameraImpl(args.device_id,args.width,args.height,args.fps)
    for _ in range(10):
        camera.capture_frame()
    
    with RR.ServerNodeSetup("com.robotraconteur.imaging.camera",59823,argv=rr_args):

        service_ctx = RRN.RegisterService("camera","com.robotraconteur.imaging.Camera",camera)



        if args.wait_signal:  
            #Wait for shutdown signal if running in service mode          
            print("Press Ctrl-C to quit...")
            import signal
            signal.sigwait([signal.SIGTERM,signal.SIGINT])
        else:
            #Wait for the user to shutdown the service
            if (sys.version_info > (3, 0)):
                input("Server started, press enter to quit...")
            else:
                raw_input("Server started, press enter to quit...")


if __name__ == "__main__":
    main()