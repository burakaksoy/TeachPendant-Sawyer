# Client for vision 
from js import self as window
from js import document
from js import print_div
from js import ImageData
from RobotRaconteur.Client import *
import numpy as np

import base64 # To convert img byte array to base64string

class ClientVision(object):
    """Client Class to access client data in a more convenient way"""
    def __init__(self, ip_plugins):
        # Service IPs
        self.ip_plugins = ip_plugins
        
        # Define Element references
        self.define_element_references()
        
        # Define Event Listeners
        self.define_event_listeners()

        # Call the async main for async opetations on browser
        self.loop_client = RR.WebLoop()

        self.restart()
        
    def restart(self):
        self.loop_client.call_soon(self.async_client_main())

    async def async_client_main(self):
        # If a new camera is selected, disconnect from plugins to re-connect
        try:
            await self.async_disconnect_from_plugins()
        except:
            pass

        # Connect to plugins
        is_connected = await self.async_connect_to_plugins()

        if is_connected:
            self.is_stop_vision = False
            # # Show the feedback image of the first available camera initially
            self.camera_fb_pipe.PacketReceivedEvent += self.show_new_frame
            # self.camera_pipe.PacketReceivedEvent += self.show_new_frame # debug
            # self.camera.async_start_streaming(None) # debug       


    async def async_connect_to_plugins(self):
        # Discover Available Cameras
        ## Discovery plugin
        print_div('Discovery plugin is connecting (camera)..<br>')
        self.url_plugin_discovery = 'rr+ws://' + self.ip_plugins + ':8896?service=Discovery'
        self.plugin_discovery = await RRN.AsyncConnectService(self.url_plugin_discovery,None,None,None,None)
        print_div('discovery plugin is connected (camera)..<br>')
        # Get the available camera connection URLs
        self.CameraConnectionURLs = await self.plugin_discovery.async_available_camera_ConnectionURLs(None)

        
        try:
            # Trying to find an available camera url -----
            self.url_cam = await self.async_select_available_camera_url(self.CameraConnectionURLs)
            print_div('Selected Camera url: '+ self.url_cam + '<br>')

            # ip_cam = 'localhost' # debug
            # port_webcam_service = '59824' #'59824' #'59823' # debug
            # self.url_cam = 'rr+ws://' + ip_cam+ ':'+ port_webcam_service +'?service=camera' # debug
            # self.camera = await RRN.AsyncConnectService(self.url_cam,None,None,None,None) # debug
            # self.camera_pipe = await self.camera.preview_stream.AsyncConnect(-1,None) # debug

            # Plugin Port numbers
            self.port_pluginCameraFeedback_service = '8889'
            self.port_pluginCameraTraining_service = '8892'
            self.port_pluginCameraCalibration_service = '8893'
            
            # Create Service and Plugin URLs 
            # rr+ws : WebSocket connection without encryption
            # self.url_cam = 'rr+ws://' + ip_cam+ ':'+ self.port_webcam_service +'?service=Webcam'
            self.url_plugin_cameraFeedback = 'rr+ws://' + self.ip_plugins + ':' + self.port_pluginCameraFeedback_service + '?service=CameraFeedback'
            self.url_plugin_cameraTraining = 'rr+ws://' + self.ip_plugins + ':' + self.port_pluginCameraTraining_service + '?service=CameraTraining'
            self.url_plugin_cameraCalibration='rr+ws://'+ self.ip_plugins + ':' + self.port_pluginCameraCalibration_service+'?service=CameraCalibration'

            ## CameraFeedback plugin
            print_div('CameraFeedback plugin is connecting..<br>')
            self.plugin_cameraFeedback = await RRN.AsyncConnectService(self.url_plugin_cameraFeedback,None,None,None,None)
            await self.plugin_cameraFeedback.async_connect2camera(self.url_cam,None)
            self.camera_fb_pipe = await self.plugin_cameraFeedback.preview_stream_out.AsyncConnect(-1,None) 
            print_div('CameraFeedback plugin is connected!<br>')

            ## CameraTraining plugin
            print_div('CameraTraining plugin is connecting..<br>')
            self.plugin_cameraTraining = await RRN.AsyncConnectService(self.url_plugin_cameraTraining,None,None,None,None)
            await self.plugin_cameraTraining.async_connect2camera(self.url_cam,None)
            print_div('CameraTraining plugin is connected!<br>')

            ## CameraCalibration plugin
            print_div('CameraCalibration plugin is connecting..<br>')
            self.plugin_cameraCalibration = await RRN.AsyncConnectService(self.url_plugin_cameraCalibration,None,None,None,None)
            # await self.plugin_cameraCalibration.async_connect2camera(self.url_cam,None)
            print_div('CameraCalibration plugin is connected!<br>')

            return True
        except:
            import traceback
            print_div(traceback.format_exc())
            print_div("No camera could be found, or something went wrong..<br>")
            return False

        

    async def async_disconnect_from_plugins(self):
        await RRN.AsyncDisconnectService(self.plugin_discovery, None)
        await self.camera_fb_pipe.AsyncClose(None)
        await RRN.AsyncDisconnectService(self.plugin_cameraFeedback, None)
        await RRN.AsyncDisconnectService(self.plugin_cameraTraining, None)
        await RRN.AsyncDisconnectService(self.plugin_cameraCalibration, None)

        # await self.camera_pipe.AsyncClose(None) # debug
        # await self.camera.async_stop_streaming(None) #debug
        # await RRN.AsyncDisconnectService(self.camera, None)  # debug
        

    def define_element_references(self):
        print_div("HTML Element references are being created..<br>")
        # For Feedback
        self.available_cams_list = document.getElementById("available_cams")
        self.img_feedback = document.getElementById("camera_image")

        # For Training
        self.button_train_new_visual = document.getElementById("train_new_visual_btn")
        self.canvas_2 = document.getElementById("selected_trained_visual")
        self.ctx_2 = self.canvas_2.getContext("2d")

    def define_event_listeners(self):
        print_div("Event Listeners are being created.. <br>")
        # For Feedback
        self.available_cams_list.addEventListener("change", self.select_available_cam_func)
        # For Training
        self.button_train_new_visual.addEventListener("click", self.train_new_visual_func)

    ## Callback functions
    # gets the selected index and return the camera url from the given camera urls
    async def async_select_available_camera_url(self, camera_urls):
        print_div("Selecting the camera URL.. <br>")
        # Read the selected camera index from the browser  
        index = self.available_cams_list.selectedIndex
        if index == -1:
            print_div("No camera is selected, Selecting the first available camera")
            return camera_urls[0]
        return camera_urls[index]

    # For Feedback
    def select_available_cam_func(self,data):
        print_div("Selecting the cam.. <br>")
        self.loop_client.call_soon(self.async_new_cam_selected())

    async def async_new_cam_selected(self):
        # Trying -----
        self.url_cam = await self.async_select_available_camera_url(self.CameraConnectionURLs)
        print_div('Selected Camera url: '+ self.url_cam + '<br>')
        # ------------
        self.is_stop_vision = True
        await RRN.AsyncSleep(0.5,None)  # Await a bit to camera feedback stops in the main 
        self.restart()

    def show_new_frame(self,pipe_ep):
        #Loop to get the newest frame
        while (pipe_ep.Available > 0):
            #Receive the packet
            image = pipe_ep.ReceivePacket()
            
            # print_div(str(len(image.data)))
            # print_div(str(image.data))
            
            encoded = str(base64.b64encode(image.data))[2:-1]
            # encoded = str(base64.standard_b64encode(image.data))
            # print_div(encoded)

            self.img_feedback.src = "data:image/jpg;base64," + encoded
            # print_div(self.img_feedback.src)
            self.img_feedback.width=str(image.image_info.width)
            self.img_feedback.height=str(image.image_info.height)
            # print_div(str(image.image_info.width) + ", " + str(image.image_info.height))


    # For Training
    def train_new_visual_func(self,data):
        print_div("Training a new image.. <br>")
        self.loop_client.call_soon(self.async_train_new_visual())

    async def async_img_for_canvas(self,image):
        # Makes "image" from RR suitable to put on Canvas as "image_data"
        # Image info
        image_info = image.image_info
        # image_info.data_header # Type: SensorDataHeader Empty For now
        height = image_info.height
        width = image_info.width
        step = image_info.step
        encoding = image_info.encoding # Type: ImageEncoding, enum for rgb8(0x1000), mono8(0x2000) etc.
        # print_div(height)
        # print_div(width)
        # print_div(step)
        # print_div(encoding)

        # data = image.data # uint8[] (array)
        # Decode image data for web
        imageBytes=np.zeros(4*width*height, dtype=np.uint8) #dtype essential here, IndexSizeError
        imageBytes[3::4] = 255
        imageBytes[0::4] = image.data[2::3]
        imageBytes[1::4] = image.data[1::3]
        imageBytes[2::4] = image.data[0::3]
        
        image_data = ImageData.new(bytes(imageBytes),width,height)        
        # Note: bytes(imageBytes): Is a Uint8ClampedArray representing a one-dimensional array containing the data in the RGBA order, with integer values between 0 and 255
        return image_data, width, height

    async def async_train_new_visual(self):
        try: 
            image = await self.plugin_cameraTraining.async_train_new_visual(None)
            image_data, sw,sh = await self.async_img_for_canvas(image)
            # await self.async_img_for_canvas(image)
            # Show image
            self.ctx_2.putImageData(image_data,0,0,0,0,320,240)
            # self.ctx.putImageData(image_data,0,0,sw,sh,0,0,320,240)
            # self.ctx.putImageData(image_data,320,240)

        except:
            import traceback
            print_div(traceback.format_exc())
            # raise


       
async def client_vision():
    # ip_cam = 'localhost'
    # ip_cam = '192.168.50.152'
    # ip_cam = '192.168.50.40'
    
    # ip_plugins = 'localhost'
    ip_plugins = '192.168.50.152'
    
    try:
        # Run the client as a class to access client data in a more convenient way
        # cli_vision = ClientVision(ip_cam,ip_plugins) 
        cli_vision = ClientVision(ip_plugins) 

        
    except:
        import traceback
        print_div(traceback.format_exc())
        raise

loop = RR.WebLoop()
loop.call_soon(client_vision())
# RR.WebLoop.run(client_vision())

# RRN.PostToThreadPool(client_vision()) 
