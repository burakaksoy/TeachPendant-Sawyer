# Client for vision 
from js import print_div
from js import document
from js import ImageData
from RobotRaconteur.Client import *
import numpy as np

class ClientVision(object):
    """Client Class to access client data in a more convenient way"""
    def __init__(self, ip_cam, ip_plugins):
        # Service IPs
        self.ip_cam = ip_cam
        self.ip_plugins = ip_plugins
        
        # Port numbers
        self.port_webcam_service = '2355'
        self.port_pluginCameraFeedback_service = '8889'
        self.port_pluginCameraTraining_service = '8892'
        self.port_pluginCameraCalibration_service = '8893'
        
        # Create Service and Plugin URLs 
        # rr+ws : WebSocket connection without encryption
        self.url_cam = 'rr+ws://' + ip_cam+ ':'+ self.port_webcam_service +'?service=Webcam'
        self.url_plugin_cameraFeedback = 'rr+ws://' + self.ip_plugins + ':' + self.port_pluginCameraFeedback_service + '?service=CameraFeedback'
        self.url_plugin_cameraTraining = 'rr+ws://' + self.ip_plugins + ':' + self.port_pluginCameraTraining_service + '?service=CameraTraining'
        self.url_plugin_cameraCalibration='rr+ws://'+ self.ip_plugins + ':' + self.port_pluginCameraCalibration_service+'?service=CameraCalibration'

        self.selCamInd = None

        # Define Element references
        self.define_element_references()
        # Define Event Listeners
        self.define_event_listeners()

        # Call the async main for async opetations on browser
        self.loop_client = RR.WebLoop()
        self.loop_client.call_soon(self.async_client_main())

    async def async_client_main(self):
        # Connect to plugins
        await self.async_connect_to_plugins()

        # WebcamHost object
        self.c_host = await RRN.AsyncConnectService(self.url_cam,None,None,None,None)

        # Get available webcam names and add them as options to available_cams in html
        await self.async_create_available_cams_list()

        # Show the feedback image of the first available camera initially
        await self.async_show_selected_cam_feedback()


    def new_frame(self,pipe_ep):
        #Loop to get the newest frame
        while (pipe_ep.Available > 0):
            #Receive the packet
            image = pipe_ep.ReceivePacket()
            #Convert the packet to an image and set the global variable
            
            # if (self.canvas == None):
            #     self.canvas = document.getElementById("camera_image")
            #     self.ctx = self.canvas.getContext("2d")
            
            imageBytes=np.zeros(4*image.width*image.height, dtype=np.uint8) #dtype essential here, IndexSizeError
            imageBytes[3::4] = 255
            imageBytes[0::4] = image.data[2::3]
            imageBytes[1::4] = image.data[1::3]
            imageBytes[2::4] = image.data[0::3]

            image_data = ImageData.new(bytes(imageBytes),image.width,image.height)
            self.ctx.putImageData(image_data, 0, 0,0,0,320,240)


    async def async_connect_to_plugins(self):
        ## CameraFeedback plugin
        print_div('CameraFeedback plugin is connecting..<br>')
        plugin_cameraFeedback = await RRN.AsyncConnectService(self.url_plugin_cameraFeedback,None,None,None,None)
        print_div('CameraFeedback plugin is connected!<br>')

        ## CameraTraining plugin
        print_div('CameraTraining plugin is connecting..<br>')
        plugin_cameraTraining = await RRN.AsyncConnectService(self.url_plugin_cameraTraining,None,None,None,None)
        print_div('CameraTraining plugin is connected!<br>')

        ## CameraCalibration plugin
        print_div('CameraCalibration plugin is connecting..<br>')
        plugin_cameraCalibration = await RRN.AsyncConnectService(self.url_plugin_cameraCalibration,None,None,None,None)
        print_div('CameraCalibration plugin is connected!<br>')


    async def async_create_available_cams_list(self):
        print_div('Creating available cameras options..<br>')
        
        # # Get ref to the html element
        # element_id = "available_cams"
        # self.available_cams_list = document.getElementById(element_id)

        self.webcam_names = await self.c_host.async_get_WebcamNames(None) # string{int32} i.e. a map (dictionary) of strings keyed by an integer.
        print_div(str(self.webcam_names) + "<br>") # str(self.webcam_names) --> {0:'Right'}
        for key in self.webcam_names:
            # print_div(str(self.webcam_names[key]) + "<br>") # --> Right
            # Add the current joint angles to the available_cams list
            option = document.createElement("option")
            option.text = str(key) + ":" + str(self.webcam_names[key])
            self.available_cams_list.add(option)

        # option = document.createElement("option")
        # option.text = str("CAM 1")
        # self.available_cams_list.add(option)
        # option = document.createElement("option")
        # option.text = str("CAM 2")
        # self.available_cams_list.add(option)


    async def async_show_selected_cam_feedback(self):
        # Get ref to the html element
        index = self.available_cams_list.selectedIndex # Dictionary Key of the camera
        print_div("selected index: "+str(index)+"<br>")

        # Connects to WebcamImage struct pipe
        if self.selCamInd is None:
            # Start new one
            self.selCamInd = index
               
            # thisdict[sorted(thisdict)[1]] 
            key = sorted(self.webcam_names)[self.selCamInd]
            print_div("selected key: "+str(key)+"<br>")

            # Selects index th Webcam object from Array of Webcam objects
            self.c = await self.c_host.async_get_Webcams(key,None)
                
            self.p = await self.c.FrameStream.AsyncConnect(-1,None)

            
            print_div("Camera '" + str(self.webcam_names[key]) +"' is Selected!<br>")

            self.p.PacketReceivedEvent += self.new_frame

            self.c.async_StartStreaming(None)           
            await RRN.AsyncSleep(0.01,None)


        elif self.selCamInd != index:
            # Shut down the previous cam feedback
            # await self.p.AsyncClose(None)
            # await self.c.FrameStream.AsyncClose(-1,None)
            await self.c.async_StopStreaming(None) 

            # Start new one 
            self.selCamInd = index

            # thisdict[sorted(thisdict)[1]] 
            key = sorted(self.webcam_names)[self.selCamInd]
            print_div("selected key: "+str(key)+"<br>")

            # Selects index th Webcam object from Array of Webcam objects
            self.c = await self.c_host.async_get_Webcams(key,None)
                
            self.p = await self.c.FrameStream.AsyncConnect(-1,None)

            print_div("Camera '" + str(self.webcam_names[key]) +"' is Selected!<br>")

            self.p.PacketReceivedEvent += self.new_frame

            self.c.async_StartStreaming(None)           
            await RRN.AsyncSleep(0.01,None)

    
    def define_element_references(self):
        print_div("HTML Element references are being created..<br>")
        self.available_cams_list = document.getElementById("available_cams")
        self.canvas = document.getElementById("camera_image")
        self.ctx = self.canvas.getContext("2d")


    def define_event_listeners(self):
        print_div("Event Listeners are being created.. <br>")
        self.available_cams_list.addEventListener("change", self.select_available_cam_func)

    # Callback functions
    def select_available_cam_func(self,data):
        print_div("Selecting the cam.. <br>")
        self.loop_client.call_soon(self.async_show_selected_cam_feedback())
       
async def client_vision():
    # ip_cam = 'localhost'
    ip_cam = '192.168.50.152'
    # ip_cam = '192.168.50.40'
    
    # ip_plugins = 'localhost'
    ip_plugins = '192.168.50.152'
    
    try:
        # Run the client as a class to access client data in a more convenient way
        cli_vision = ClientVision(ip_cam,ip_plugins) 
        
    except:
        import traceback
        print_div(traceback.format_exc())
        raise

loop = RR.WebLoop()
loop.call_soon(client_vision())
# RR.WebLoop.run(client_vision())

# RRN.PostToThreadPool(client_vision()) 
