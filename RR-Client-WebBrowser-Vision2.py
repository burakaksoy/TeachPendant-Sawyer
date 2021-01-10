# Client for vision 
from js import self as window
from js import document
from js import print_div
from js import print_div_test_selected_camera
from js import print_div_test_selected_visual
from js import print_div_test_selected_camera_img_size
from js import print_div_test_selected_visual_img_size
from js import print_div_test_detected_obj_size
from js import print_div_test_detected_obj_coordinates
from js import print_div_test_detected_obj_angle

from js import ImageData
from RobotRaconteur.Client import *
import numpy as np

import base64 # To convert img byte array to base64string

from js import Cropper
from js import Blockly
from js import register_vision_extensions_blockly

class ClientVision(object):
    """Client Class to access client data in a more convenient way"""
    def __init__(self, ip_plugins):

        # Service IPs
        self.ip_plugins = ip_plugins

        # Training image avatar sizes        
        self.h_avatar = 160
        self.w_avatar = 160

        # Register Extensions of Blockly Camera Blocks (see blockly-customBlocks/camera_blocks.js)
        register_vision_extensions_blockly()
        self.camera_node_names_lst = None # For storing camera names
        self.image_files_lst = None # For storing trained image file names

        # RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.image'],True)
        # self._image_consts = RRN.GetConstants('com.robotraconteur.image')
        # self._image_type = RRN.GetStructureType('com.robotraconteur.image.Image')
        # self._image_info_type = RRN.GetStructureType('com.robotraconteur.image.ImageInfo')

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
        self.camera_node_names_lst =  await self.plugin_discovery.async_available_camera_NodeNames(None)
        
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
            self.port_pluginCameraTracking_service = '8898'

            self.port_pluginBlockly_service = '8897'
            
            # Create Service and Plugin URLs 
            # rr+ws : WebSocket connection without encryption
            # self.url_cam = 'rr+ws://' + ip_cam+ ':'+ self.port_webcam_service +'?service=Webcam'
            self.url_plugin_cameraFeedback = 'rr+ws://' + self.ip_plugins + ':' + self.port_pluginCameraFeedback_service + '?service=CameraFeedback'
            self.url_plugin_cameraTraining = 'rr+ws://' + self.ip_plugins + ':' + self.port_pluginCameraTraining_service + '?service=CameraTraining'
            self.url_plugin_cameraCalibration = 'rr+ws://'+ self.ip_plugins + ':' + self.port_pluginCameraCalibration_service+'?service=CameraCalibration'
            self.url_plugin_cameraTracking = 'rr+ws://'+ self.ip_plugins + ':' + self.port_pluginCameraTracking_service+'?service=CameraTracking'
            
            self.url_plugin_blockly = 'rr+ws://'+ self.ip_plugins + ':' + self.port_pluginBlockly_service+'?service=Blockly'


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
            await self.async_update_saved_images()
            await self.async_select_trained_visual()
            print_div('CameraTraining plugin is connected!<br>')

            ## CameraCalibration plugin
            print_div('CameraCalibration plugin is connecting..<br>')
            self.plugin_cameraCalibration = await RRN.AsyncConnectService(self.url_plugin_cameraCalibration,None,None,None,None)
            # await self.plugin_cameraCalibration.async_connect2camera(self.url_cam,None)
            print_div('CameraCalibration plugin is connected!<br>')

            ## CameraTracking plugin
            print_div('CameraTracking plugin is connecting..<br>')
            self.plugin_cameraTracking = await RRN.AsyncConnectService(self.url_plugin_cameraTracking,None,None,None,None)
            self.url_plugins_vision_lst = [self.url_plugin_cameraFeedback,self.url_plugin_cameraTraining,self.url_plugin_cameraCalibration]
            # Make camera tracking connect to all vision plugins as well so that it can reach the inner files of those services with their permit
            await self.plugin_cameraTracking.async_connect2plugins_vision(self.url_plugins_vision_lst,None) 
            # Make tracking plugin to connect to all cameras and make it get the all corresponding camera (node) names
            await self.plugin_cameraTracking.async_connect2all_cameras(self.CameraConnectionURLs,self.camera_node_names_lst,None) 
            print_div('CameraTrackin plugin is connected!<br>')

            ## Blockly plugin
            print_div('Blockly plugin is connecting (from Vision)..<br>')
            self.plugin_blockly = await RRN.AsyncConnectService(self.url_plugin_blockly,None,None,None,None)
            self.url_plugins_vision_lst = [self.url_plugin_cameraFeedback,self.url_plugin_cameraTraining,self.url_plugin_cameraCalibration,self.url_plugin_cameraTracking ]
            await self.plugin_blockly.async_connect2plugins_vision(self.url_plugins_vision_lst,None)            
            print_div('Blockly plugin is connected (from Vision)!<br>')


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
        self.img_selected_trained_visual = document.getElementById("selected_trained_visual")

        self.img_training = document.getElementById("training_image")
        self.modal = document.getElementById("myModal");

        self.span_close = document.getElementById("span_close")
        self.button_close = document.getElementById("btn_close")

        self.button_crop = document.getElementById("btn_crop")
        self.button_reset = document.getElementById("btn_reset")
        self.button_rot_m5 = document.getElementById("btn_rot_p5")
        self.button_rot_p5 = document.getElementById("btn_rot_m5")

        self.select_trained_visuals = document.getElementById("trained_visuals")

        self.button_delete_selected_trained_visual = document.getElementById("delete_selected_trained_visual_btn")
        self.button_edit_name_selected_trained_visual = document.getElementById("edit_name_selected_trained_visual_btn")

        # For Object Detection Tracking
        self.button_test_detection = document.getElementById("test_detection_btn")
        self.img_test_detection_result = document.getElementById("test_detection_result_image")


    def define_event_listeners(self):
        print_div("Event Listeners are being created.. <br>")
        # For Feedback
        self.available_cams_list.addEventListener("change", self.select_available_cam_func)

        
        # For Training
        self.button_train_new_visual.addEventListener("click", self.train_new_visual_func)
        
        self.span_close.addEventListener("click", self.close_modal_func)
        self.button_close.addEventListener("click", self.close_modal_func)
        # window.addEventListener("click", self.close_modal_func)

        self.button_crop.addEventListener("click", self.crop_img_func)
        self.button_reset.addEventListener("click", self.reset_img_func)
        self.button_rot_m5.addEventListener("click", self.rot_m5_img_func)
        self.button_rot_p5.addEventListener("click", self.rot_p5_img_func)

        self.select_trained_visuals.addEventListener("change", self.select_trained_visual_func)
        self.button_delete_selected_trained_visual.addEventListener("click", self.delete_selected_trained_visual_func)
        self.button_edit_name_selected_trained_visual.addEventListener("click", self.edit_name_selected_trained_visual_func)


        # For Object Detection Tracking
        self.button_test_detection.addEventListener("click", self.test_detection_func)

    # # -------------------------- BEGIN: Callback functions -------------------------- #
    # gets the selected index and return the camera url from the given camera urls
    async def async_select_available_camera_url(self, camera_urls):
        print_div("Selecting the camera URL.. <br>")
        # Read the selected camera index from the browser  
        index = self.available_cams_list.selectedIndex
        if index == -1:
            print_div("No camera is selected, Selecting the first available camera")
            return camera_urls[0]
        return camera_urls[index]

    # Callback functions For FEEDBACK
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
            self.img_feedback.width= str(image.image_info.width)
            self.img_feedback.height= str(image.image_info.height)
            # print_div(str(image.image_info.width) + ", " + str(image.image_info.height))


    # Callback functions For TRAINING
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
            # print_div(image_data)

            # Add image for cropping
            canvas = document.createElement('canvas')
            ctx = canvas.getContext('2d')
            canvas.width = sw
            canvas.height = sh
            ctx.putImageData(image_data, 0, 0)

            self.img_training.src = canvas.toDataURL()
            
            # Show the modal
            self.modal.style.display = "block"

            self.cropper = Cropper.new(self.img_training, {"viewMode":2}) # TODO options, Done

        except:
            import traceback
            print_div(traceback.format_exc())
            # raise

    def close_modal_func(self,data):
        self.modal.style.display = "none"
        if self.cropper != None:
            self.cropper.destroy()
            self.cropper = None

    def crop_img_func(self,data):
        loop.call_soon(async_save_image())

    def crop_img_func(self,data):
        print_div("Crop & Save button is clicked!<br>")
        # Hide cropping modal
        self.modal.style.display = "none"
        
        # # Get the cropped image avatar
        # self.cropped_canvas_avatar = self.cropper.getCroppedCanvas({"width": self.w_avatar,"height": self.h_avatar}) 
        # # Show the cropped image avatar
        # self.img_selected_trained_visual.src = self.cropped_canvas_avatar.toDataURL()
        # self.img_selected_trained_visual.width = str(self.w_avatar)
        # self.img_selected_trained_visual.height = str(self.h_avatar)

        # Save the cropped image for offline usage
        fileName = window.prompt("Please enter the new file name to save image (Don't forget .png file extension)", "cropped001.png")
        if (fileName != None and fileName != ""):
            # Search the existing filenames to check for coincidence
            is_found = fileName in self.image_files_lst

            if not is_found:
                # Get the cropped image
                self.cropped_canvas = self.cropper.getCroppedCanvas({"imageSmoothingEnabled":False})
                # Save the image
                self.loop_client.call_soon(self.async_save_image(fileName))
                
            else:
                window.alert("The file already exists! Try another file name")    
        else:
            window.alert("Enter a valid file name and try again.." )

        self.cropper.destroy()
        self.cropper = None

    async def async_canvas_to_image_str(self, canvas):
        data_url = canvas.toDataURL('image/png')
        # print_div(canvas.toDataURL('image/png'))

        # Remove the unnecessary part from the image string
        img_str = data_url.replace("data:image/png;base64,","")
        # print_div(img_str)
        # print_div(len(img_str))
        
        # print_div(canvas.width)
        # print_div(canvas.height)

        # image.data = mat.reshape(mat.size, order='C')
        return img_str,canvas.width, canvas.height
        
    async def async_save_image(self,fileName):
        # Convert the cropped image to RR Image type
        img_str,w,h = await self.async_canvas_to_image_str(self.cropped_canvas)
        
        # Send this image to training service plug in to save
        await self.plugin_cameraTraining.async_save_image(fileName,img_str,w,h, None)

        # # Saves with browser
        # link = document.createElement("a")
        # document.body.appendChild(link) # for Firefox
        # # Download the image
        # base64_ = self.cropped_canvas.toDataURL()
        # link.href = base64_
        # link.download = fileName
        # link.click()

        # Update the available saved images list
        await self.async_update_saved_images()
        await self.async_select_trained_visual()

    def reset_img_func(self,data):
        self.cropper.reset()

    def rot_m5_img_func(self,data):
        self.cropper.rotate(-5)

    def rot_p5_img_func(self,data):
        self.cropper.rotate(5)

    async def async_update_saved_images(self):
        try:
            # print_div("Clearing the previous available visual options..")
            length = self.select_trained_visuals.options.length
            i = length-1
            while i >= 0:
                # self.select_trained_visuals.options[i] = None
                self.select_trained_visuals.remove(i)
                i -= 1

            print_div('Creating available trained images options..<br>')
            self.image_files_lst = await self.plugin_cameraTraining.async_saved_images(None)
            # print_div(str(self.image_files_lst) + "<br>") 
            i = 0
            for fileName in self.image_files_lst:
                # print_div(str(self.camera_nodeNames[key]) + "<br>") # --> Right
                # Add the available image fileName to the select_trained_visuals list
                option = document.createElement("option")
                option.text = str(i) + ": " + str(fileName)
                self.select_trained_visuals.add(option)
                i += 1 

            self.rerender_workspace_blocks()
        except:
            import traceback
            print_div(traceback.format_exc())

    def select_trained_visual_func(self,data):
        print_div("A Saved image is selected!<br>")
        self.loop_client.call_soon(self.async_select_trained_visual())

    async def async_select_trained_visual(self):
        # Read the selected image index from the browser
        index = self.select_trained_visuals.selectedIndex
        try:
            if index == -1 and len(self.image_files_lst) == 0:
                print_div("No available trained images found")
                # Clear the selected image area
                self.img_selected_trained_visual.src = "''"

            else:
                if index == -1 and len(self.image_files_lst)> 0:
                    print_div("Selecting the first available trained image")
                    index = 0

                file_name = self.image_files_lst[index]
                print_div(file_name)
                image = await self.plugin_cameraTraining.async_load_image(file_name,None)
                image_data, sw,sh = await self.async_img_for_canvas(image)

                canvas = document.createElement('canvas')
                ctx = canvas.getContext('2d')
                canvas.width = sw
                canvas.height = sh
                ctx.putImageData(image_data, 0, 0)

                self.img_selected_trained_visual.src = canvas.toDataURL()
                self.img_selected_trained_visual.width = str(self.w_avatar)
                self.img_selected_trained_visual.height = str(self.h_avatar)
        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
            pass

        # Update the available saved images list
        # await self.async_update_saved_images()

    def delete_selected_trained_visual_func(self,data):
        print_div("Delete selected_trained_visual button is clicked!<br>")
        self.loop_client.call_soon(self.async_delete_selected_trained_visual())

    async def async_delete_selected_trained_visual(self):
        # Read the selected image index from the browser
        index = self.select_trained_visuals.selectedIndex
        try:
            if index == -1 or len(self.image_files_lst) == 0:
                print_div("No available trained images found or not selected")

            else:
                # if index == -1 and len(self.image_files_lst)> 0:
                #     print_div("Selecting the first available trained image")
                #     index = 0

                file_name = self.image_files_lst[index]
                # Ask user: are you sure you want to delete 'file_name'
                resp = window.confirm("Sure you want to delete '" + file_name + "'?")
                # if sure delete, else ignore and return
                if resp:
                    await self.plugin_cameraTraining.async_delete_image(file_name,None)
        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
            pass

        # Update the available saved images list
        await self.async_update_saved_images()
        await self.async_select_trained_visual()

    def edit_name_selected_trained_visual_func(self,data):
        print_div("Edit selected_trained_visual name button is clicked!<br>")
        self.loop_client.call_soon(self.async_edit_name_selected_trained_visual())

    async def async_edit_name_selected_trained_visual(self):
        # Read the selected image index from the browser
        index = self.select_trained_visuals.selectedIndex
        try:
            if index == -1 or len(self.image_files_lst) == 0:
                print_div("No available trained images found or not selected")

            else:
                # if index == -1 and len(self.image_files_lst)> 0:
                #     print_div("Selecting the first available trained image")
                #     index = 0

                file_name = self.image_files_lst[index]

                # Ask user: New name for 'file_name'
                file_name_new = window.prompt("Please enter the new file name (Don't forget .png file extension)", file_name);
                if (file_name_new != None and file_name_new != ""):
                    await self.plugin_cameraTraining.async_edit_image_name(file_name,file_name_new, None)
        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
            pass

        # Update the available saved images list
        await self.async_update_saved_images()
        await self.async_select_trained_visual()

    # Callback functions For OBJECT DETECTION TRACKING
    def test_detection_func(self,data):
        print_div("test_detection button is clicked!<br>")
        self.loop_client.call_soon(self.async_test_detection())

    async def async_test_detection(self):
        try:
            # Print selected camera name
            index = self.available_cams_list.selectedIndex

            if index == -1 or len(self.available_cams_list) == 0:
                print_div_test_selected_camera("No available camera is found or none selected")
            else:
                camera_name = self.camera_node_names_lst[index]
                print_div_test_selected_camera(camera_name)

            # Print Selected visual file name
            index = self.select_trained_visuals.selectedIndex
            if index == -1 or len(self.image_files_lst) == 0:
                print_div_test_selected_visual("No available trained images found or not selected")
            else:
                obj_img_filename = self.image_files_lst[index]
                print_div_test_selected_visual(obj_img_filename)

            # Execute the object detection
            return_result_image = True
            detection_result = await self.plugin_cameraTracking.async_find_object_in_img_frame(obj_img_filename, camera_name, return_result_image, None)

            #Show the results
            image = detection_result.result_img
            encoded = str(base64.b64encode(image.data))[2:-1]
            # encoded = str(base64.standard_b64encode(image.data))
            # print_div(encoded)

            self.img_test_detection_result.src = "data:image/jpg;base64," + encoded
            # print_div(self.img_test_detection_result.src)
            self.img_test_detection_result.width= str(image.image_info.width)
            self.img_test_detection_result.height= str(image.image_info.height)
            # print_div(str(image.image_info.width) + ", " + str(image.image_info.height))

            # print_div_test_selected_camera_img_size("")
            # print_div_test_selected_visual_img_size("")
            print_div_test_detected_obj_size(str(detection_result.width) + "," + str(detection_result.height))
            print_div_test_detected_obj_coordinates(str(detection_result.center_x) + "," + str(detection_result.center_y))
            print_div_test_detected_obj_angle(str(detection_result.angle) + " degrees")

        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
            pass
        

    # # -------------------------- END: Callback functions -------------------------- #


    # # ---------------------------BEGIN: BLOCKLY FUNCTIONS  --------------------------- #
    # Extension Registration Functions BEGIN
    def image_files_lst_for_blockly(self):
        if self.image_files_lst is None or len(self.image_files_lst) < 1:
            optionss = [["none","NONE"]]
        else:
            optionss = [] # Create an empty list
            for fileName in self.image_files_lst:
                # optionss.append([fileName, fileName.upper()])
                optionss.append([fileName, fileName])
                # print_div(str([fileName, fileName.upper()]))

        return optionss

    def camera_node_names_lst_for_blockly(self):
        if self.camera_node_names_lst is None or len(self.camera_node_names_lst) < 1:
            optionss = [["none","NONE"]]
        else:
            optionss = [] # Create an empty list
            for nodeName in self.camera_node_names_lst:
                # optionss.append([nodeName, nodeName.upper()])
                optionss.append([nodeName, nodeName])
                # print_div(str([nodeName, nodeName.upper()]))

        return optionss
    # Extension Registration Functions END

    def rerender_workspace_blocks(self):
        # This function refreshes the blockly workspace but highly inefficiently.
        # Further info: https://github.com/google/blockly/issues/3941

        workspace = Blockly.getMainWorkspace() # get the Blockly workspace
        workspace_xml = Blockly.Xml.workspaceToDom(workspace)
        workspace.clear()
        Blockly.Xml.domToWorkspace(workspace_xml, workspace)
        workspace.refreshToolboxSelection();







    # # ---------------------------END: BLOCKLY FUNCTIONS --------------------------- #


async def client_vision():
    # ip_cam = 'localhost'
    # ip_cam = 'localhost'
    # ip_cam = '192.168.50.40'
    
    # ip_plugins = 'localhost'
    ip_plugins = 'localhost'
    
    try:
        # Run the client as a class to access client data in a more convenient way
        # cli_vision = ClientVision(ip_cam,ip_plugins) 
        global cli_vision
        cli_vision = ClientVision(ip_plugins) 

        
    except:
        import traceback
        print_div(traceback.format_exc())
        raise

loop = RR.WebLoop()
loop.call_soon(client_vision())
# RR.WebLoop.run(client_vision())

# RRN.PostToThreadPool(client_vision()) 