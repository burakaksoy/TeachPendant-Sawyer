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

from js import print_div_selected_camera_matrix
from js import print_div_selected_camera_distortion_coefficients
from js import print_div_selected_camera_RnT
from js import print_div_num_captured_calibration_imgs
from js import print_div_selected_camera_calibration_error
from js import clear_div_modal_body_CameraCalibration

from js import print_div_selected_robotcamerapair_rotation_matrix
from js import print_div_selected_robotcamerapair_translation_vector
from js import print_div_robotcamera_calibration_selected_robot
from js import print_div_robotcamera_calibration_selected_camera
from js import print_div_num_captured_robotcamera_calibration_imgs
from js import clear_div_modal_body_RobotCameraCalibration
from js import copy_saved_poses
from js import remove_copied_saved_poses

from js import ImageData
from RobotRaconteur.Client import *
import numpy as np
import asyncio
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
        self.calibration_files_lst = None # For storing the camera calibration file names
        self.robotcamera_calibration_files_lst = None # For storing the robot-camera calibration file names

        self.camera_name = None # To store currently selected (active) camera name
        self.robot_name = None # To store currently selected (active) robot name

        self.is_updated_available_cameras = False # Created to update the available cameras only once 

        # RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.image'],True)
        # self._image_consts = RRN.GetConstants('com.robotraconteur.image')
        # self._image_type = RRN.GetStructureType('com.robotraconteur.image.Image')
        # self._image_info_type = RRN.GetStructureType('com.robotraconteur.image.ImageInfo')

        # Define Element references
        self.define_element_references()
        # Define Event Listeners
        self.define_event_listeners()

        # Call the async main for async opetations on browser
        self.loop_client = asyncio.get_event_loop()

        self.restart()
        
    def restart(self):
        self.loop_client.create_task(self.async_client_main())

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
     

    async def async_connect_to_plugins(self):
        # Discover Available Cameras
        ## Discovery plugin
        print_div('Discovery plugin is connecting (camera)..<br>')
        self.url_plugin_discovery = 'rr+ws://' + self.ip_plugins + ':8896?service=Discovery'
        self.plugin_discovery = await RRN.AsyncConnectService(self.url_plugin_discovery,None,None,None,None)
        print_div('discovery plugin is connected (camera)..<br>')
        
        try:
            # Plugin Port numbers
            self.port_pluginCameraFeedback_service = '8889'
            self.port_pluginCameraTraining_service = '8892'
            self.port_pluginCameraCalibration_service = '8893'
            self.port_pluginCameraTracking_service = '8898'
            self.port_pluginCameraRobotCalibration_service = '8902'

            self.port_pluginBlockly_service = '8897'

            self.port_pluginSavePlayback_service = '8894'
            
            # Create Service and Plugin URLs 
            # rr+ws : WebSocket connection without encryption
            # self.url_cam = 'rr+ws://' + ip_cam+ ':'+ self.port_webcam_service +'?service=Webcam'
            self.url_plugin_cameraFeedback = 'rr+ws://' + self.ip_plugins + ':' + self.port_pluginCameraFeedback_service + '?service=CameraFeedback'
            self.url_plugin_cameraTraining = 'rr+ws://' + self.ip_plugins + ':' + self.port_pluginCameraTraining_service + '?service=CameraTraining'
            self.url_plugin_cameraCalibration = 'rr+ws://'+ self.ip_plugins + ':' + self.port_pluginCameraCalibration_service+'?service=CameraCalibration'
            self.url_plugin_cameraTracking = 'rr+ws://'+ self.ip_plugins + ':' + self.port_pluginCameraTracking_service+'?service=CameraTracking'
            self.url_plugin_cameraRobotCalibration = 'rr+ws://'+ self.ip_plugins + ':' + self.port_pluginCameraRobotCalibration_service+'?service=CameraRobotCalibration'
            
            self.url_plugin_blockly = 'rr+ws://'+ self.ip_plugins + ':' + self.port_pluginBlockly_service+'?service=Blockly'

            self.url_plugin_savePlayback = 'rr+ws://'+ self.ip_plugins + ':' + self.port_pluginSavePlayback_service +'?service=SavePlayback'


            ## CameraFeedback plugin
            print_div('CameraFeedback plugin is connecting..<br>')
            self.plugin_cameraFeedback = await RRN.AsyncConnectService(self.url_plugin_cameraFeedback,None,None,None,None)
            print_div(self.is_updated_available_cameras)
            if not self.is_updated_available_cameras:
                await self.async_update_available_cameras() 
            await self.async_select_available_camera() 

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
            await self.plugin_cameraCalibration.async_connect2camera(self.url_cam,None)
            await self.async_update_calibrated_cameras()
            await self.async_select_calibrated_camera()
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

            ## CameraRobotCalibration plugin
            print_div('CameraCalibration plugin is connecting..<br>')
            self.plugin_cameraRobotCalibration = await RRN.AsyncConnectService(self.url_plugin_cameraRobotCalibration,None,None,None,None)
            await self.plugin_cameraRobotCalibration.async_connect2camera(self.url_cam,None)
            await self.async_update_calibrated_camerarobot_pairs()
            await self.async_select_calibrated_robotcamerapair()
            print_div('CameraRobotCalibration plugin is connected!<br>')

            ## Blockly plugin
            print_div('Blockly plugin is connecting (from Vision)..<br>')
            self.plugin_blockly = await RRN.AsyncConnectService(self.url_plugin_blockly,None,None,None,None)
            self.url_plugins_vision_lst = [self.url_plugin_cameraFeedback,self.url_plugin_cameraTraining,self.url_plugin_cameraCalibration,self.url_plugin_cameraTracking,self.url_plugin_cameraRobotCalibration ] 
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
        await RRN.AsyncDisconnectService(self.plugin_cameraTracking, None)
        await RRN.AsyncDisconnectService(self.plugin_cameraRobotCalibration, None)

        await RRN.AsyncDisconnectService(self.plugin_blockly, None)
        await RRN.AsyncDisconnectService(self.plugin_savePlayback, None)

    async def async_disconnect_from_plugin_savePlayback(self):
        await RRN.AsyncDisconnectService(self.plugin_savePlayback, None)        
        
    def define_element_references(self):
        print_div("HTML Element references are being created..<br>")
        # For Camera Feedback
        self.available_cams_list = document.getElementById("available_cams")
        self.img_feedback = document.getElementById("camera_image")

        # For Visual (Object) Training
        self.button_train_new_visual = document.getElementById("train_new_visual_btn")
        self.img_selected_trained_visual = document.getElementById("selected_trained_visual")

        self.img_training = document.getElementById("training_image")
        self.modal_TrainVision = document.getElementById("modal_TrainVision")
        self.Gl_lm_splitters = document.getElementsByClassName("lm_splitter")

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

        # For Camera Calibration
        self.select_calibrated_cameras = document.getElementById("calibrated_cameras")

        self.button_calibrate_camera = document.getElementById("calibrate_camera_btn")
        self.button_delete_calibration = document.getElementById("delete_calibration_btn")

        self.modal_CameraCalibration = document.getElementById("modal_CameraCalibration")
        self.span_close_calibration = document.getElementById("span_close_calibration")
        self.button_close_calibration = document.getElementById("btn_close_calibration")

        self.input_width_checkerboard = document.getElementById("width_checkerboard_input")
        self.input_height_checkerboard = document.getElementById("height_checkerboard_input")
        self.input_square_size = document.getElementById("square_size_input")

        self.button_capture_img_calibration = document.getElementById("capture_img_calibration_btn")
        self.img_calibration_camera = document.getElementById("calibration_camera_image")

        self.button_calibrate_n_save = document.getElementById("btn_calibrate_n_save")

        # For ROBOT-Camera Calibration
        self.select_calibrated_robotcamerapairs = document.getElementById("calibrated_robotcamerapairs")

        self.button_calibrate_robotcamerapair = document.getElementById("calibrate_robotcamerapair_btn")
        self.button_delete_robotcamerapair_calibration = document.getElementById("delete_robotcamerapair_calibration_btn")

        self.modal_RobotCameraCalibration = document.getElementById("modal_RobotCameraCalibration")
        self.span_close_robotcamera_calibration = document.getElementById("span_close_robotcamera_calibration")
        self.button_close_robotcamera_calibration = document.getElementById("btn_close_robotcamera_calibration")

        self.select_aruco_marker_dictionary = document.getElementById("dropdown_aruco_marker_dictionary")        
        self.input_arucomarker_id = document.getElementById("arucomarker_id_input")
        self.input_arucomarker_size = document.getElementById("arucomarker_size_input")
        self.input_eef_to_aruco_Tx = document.getElementById("eef_to_aruco_Tx_input")
        self.input_eef_to_aruco_Ty = document.getElementById("eef_to_aruco_Ty_input")
        self.input_eef_to_aruco_Tz = document.getElementById("eef_to_aruco_Tz_input")
        self.input_eef_to_aruco_Rx = document.getElementById("eef_to_aruco_Rx_input")
        self.input_eef_to_aruco_Ry = document.getElementById("eef_to_aruco_Ry_input")
        self.input_eef_to_aruco_Rz = document.getElementById("eef_to_aruco_Rz_input")
        # self.input_sweep_angle_j1_min = document.getElementById("sweep_angle_j1_min_input")
        # self.input_sweep_angle_j1_max = document.getElementById("sweep_angle_j1_max_input")

        self.button_jog_sel_pose = document.getElementById("jog_sel_pose_btn")
        self.button_jog_prev_pose = document.getElementById("jog_prev_pose_btn")
        self.button_jog_next_pose = document.getElementById("jog_next_pose_btn")    
        self.button_capture_img_robotcamera_calibration = document.getElementById("capture_img_robotcamera_calibration_btn")
        self.button_stop_robot = document.getElementById("stop_robot_btn")

        self.img_calibration_robotcamera = document.getElementById("calibration_robotcamera_image")

        self.button_robotcamera_calibrate_n_save = document.getElementById("btn_robotcamera_calibrate_n_save")




    def define_event_listeners(self):
        print_div("Event Listeners are being created.. <br>")
        # For Feedback
        self.available_cams_list.addEventListener("change", self.select_available_camera_func)

        
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


        # For Camera Calibration
        self.button_calibrate_camera.addEventListener("click", self.calibrate_camera_func)

        self.span_close_calibration.addEventListener("click", self.close_modal_calibration_func)
        self.button_close_calibration.addEventListener("click", self.close_modal_calibration_func)

        self.button_capture_img_calibration.addEventListener("click", self.capture_img_calibration_func)
        self.button_calibrate_n_save.addEventListener("click", self.calibrate_n_save_func)

        self.select_calibrated_cameras.addEventListener("change", self.select_calibrated_camera_func)
        self.button_delete_calibration.addEventListener("click", self.delete_calibration_func)

        # For ROBOT-Camera Calibration
        self.button_calibrate_robotcamerapair.addEventListener("click", self.calibrate_robotcamera_func)

        self.span_close_robotcamera_calibration.addEventListener("click", self.close_modal_robotcamera_calibration_func)
        self.button_close_robotcamera_calibration.addEventListener("click", self.close_modal_robotcamera_calibration_func)

        self.button_jog_sel_pose.addEventListener("click", self.jog_sel_pose_func)
        self.button_jog_prev_pose.addEventListener("click", self.jog_prev_pose_func)
        self.button_jog_next_pose.addEventListener("click", self.jog_next_pose_func)
        self.button_capture_img_robotcamera_calibration.addEventListener("click", self.capture_img_robotcamera_calibration_func)
        self.button_stop_robot.addEventListener("click", self.stop_robot_func)

        self.button_robotcamera_calibrate_n_save.addEventListener("click", self.robotcamera_calibrate_n_save_func)

        self.select_calibrated_robotcamerapairs.addEventListener("change", self.select_calibrated_robotcamerapair_func)
        self.button_delete_robotcamerapair_calibration.addEventListener("click", self.delete_robotcamerapair_calibration_func)


    # # -------------------------- BEGIN: Callback functions -------------------------- #
    # Callback functions For FEEDBACK
    async def async_update_available_cameras(self):
        try:
            # print_div("Clearing the previous available camera options..")
            length = self.available_cams_list.options.length
            i = length-1
            while i >= 0:
                self.available_cams_list.remove(i)
                i -= 1

            print_div('Creating available cameras options..<br>')
            # Get the available camera connection URLs
            self.CameraConnectionURLs = await self.plugin_discovery.async_available_camera_ConnectionURLs(None)
            self.camera_node_names_lst =  await self.plugin_discovery.async_available_camera_NodeNames(None)
            # print_div(str(self.camera_node_names_lst) + "<br>") 
            i = 0
            for cameraName in self.camera_node_names_lst:
                # Add the available cameraName to the self.available_cams_list list
                option = document.createElement("option")
                option.text = str(i) + ": " + str(cameraName)
                self.available_cams_list.add(option)
                i += 1 

            self.rerender_workspace_blocks()

            self.is_updated_available_cameras = True
        except:
            import traceback
            print_div(traceback.format_exc())

    def select_available_camera_func(self,data):
        print_div("Selecting the cam.. <br>")
        self.restart()

    async def async_select_available_camera(self):
        # Read the selected camera index from the browser
        index = self.available_cams_list.selectedIndex
        try:
            if index == -1 and len(self.camera_node_names_lst) == 0:
                print_div("No available cameras found")
                self.camera_name = None
                # Clear the selected camera image area
                self.img_feedback.src = "''"

            else:
                if index == -1 and len(self.camera_node_names_lst)> 0:
                    print_div("Selecting the first available camera")
                    index = 0

                self.camera_name = self.camera_node_names_lst[index]
                print_div("Selected camera name: " + self.camera_name + "<br>")

                self.url_cam = self.CameraConnectionURLs[index]
                print_div('Selected Camera url: '+ self.url_cam + '<br>')

                self.is_stop_vision = True
                await RRN.AsyncSleep(0.5,None)  # Await a bit to camera feedback stops in the main 

                await self.plugin_cameraFeedback.async_connect2camera(self.url_cam,None)
                self.camera_fb_pipe = await self.plugin_cameraFeedback.preview_stream_out.AsyncConnect(-1,None)

                # Show the feedback image of the first available camera initially
                self.camera_fb_pipe.PacketReceivedEvent += self.show_new_frame

                # self.restart()
        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
            pass

        # Update the available cameras list
        # await self.async_update_available_cameras()

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

            # Add camera feedback image previews into opened camera calibration modal 
            self.img_calibration_camera.src = self.img_feedback.src
            self.img_calibration_camera.width = self.img_feedback.width
            self.img_calibration_camera.height = self.img_feedback.height

            # Add camera feedback image previews into opened ROBOT-camera calibration modal 
            self.img_calibration_robotcamera.src = self.img_feedback.src
            self.img_calibration_robotcamera.width = self.img_feedback.width
            self.img_calibration_robotcamera.height = self.img_feedback.height


    # Callback functions For TRAINING
    def train_new_visual_func(self,data):
        print_div("Training a new image.. <br>")
        self.loop_client.create_task(self.async_train_new_visual())

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
            self.modal_TrainVision.style.display = "block"
            # # Hide GL splitters
            # self.disable_GL_splitters()

            self.cropper = Cropper.new(self.img_training, {"viewMode":2})

        except:
            import traceback
            print_div(traceback.format_exc())
            # raise

    # def enable_GL_splitters(self):
    #     for splitter in self.Gl_lm_splitters:
    #         # Set back to default GL splitters
    #         splitter.style.visibility = ""

    # def disable_GL_splitters(self):
    #     for splitter in self.Gl_lm_splitters:
    #         # Hide GL splitters
    #         splitter.style.visibility = "hidden"        

    def close_modal_func(self,data):
        # Hide the modal
        self.modal_TrainVision.style.display = "none"
        # # Set back to default GL splitters
        # self.enable_GL_splitters()

        if self.cropper != None:
            self.cropper.destroy()
            self.cropper = None


    def crop_img_func(self,data):
        print_div("Crop & Save button is clicked!<br>")
        # Hide cropping modal
        self.modal_TrainVision.style.display = "none"
        # # Set back to default GL splitters
        # self.enable_GL_splitters()
        
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
                self.loop_client.create_task(self.async_save_image(fileName))
                
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
        self.loop_client.create_task(self.async_select_trained_visual())

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
        self.loop_client.create_task(self.async_delete_selected_trained_visual())

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
        self.loop_client.create_task(self.async_edit_name_selected_trained_visual())

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
        self.loop_client.create_task(self.async_test_detection())

    async def async_test_detection(self):
        try:
            # Print selected camera name
            index = self.available_cams_list.selectedIndex

            if index == -1 or len(self.available_cams_list) == 0:
                print_div_test_selected_camera("No available camera is found or none selected")
                self.camera_name = None
            else:
                self.camera_name = self.camera_node_names_lst[index]
                print_div_test_selected_camera(self.camera_name)

            # Print Selected visual file name
            index = self.select_trained_visuals.selectedIndex
            if index == -1 or len(self.image_files_lst) == 0:
                print_div_test_selected_visual("No available trained images found or not selected")
            else:
                obj_img_filename = self.image_files_lst[index]
                print_div_test_selected_visual(obj_img_filename)

            # Execute the object detection
            return_result_image = True
            detection_result = await self.plugin_cameraTracking.async_find_object_in_img_frame(obj_img_filename, self.camera_name, return_result_image, None)

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

    # Callback functions for CAMERA CALIBRATION
    def calibrate_camera_func(self,data):
        print_div("Calibrating a new camera button is pressed.. <br>")
        self.loop_client.create_task(self.async_calibrate_camera())

    async def async_calibrate_camera(self):
        try: 
            # Show the modal
            self.modal_CameraCalibration.style.display = "block"
            # # Hide GL splitters
            # self.disable_GL_splitters()

            # Clear the calibration serverside captured images
            await self.plugin_cameraCalibration.async_remove_captured_images(None)
            # Get how many images are taken in the server side of calibration
            num_images = await self.plugin_cameraCalibration.async_num_of_captured_images(None)
            # Print num of images for the user see
            print_div_num_captured_calibration_imgs(str(num_images))
        except:
            import traceback
            print_div(traceback.format_exc())
            # raise

    def close_modal_calibration_func(self,data):
        # Hide the modal
        self.modal_CameraCalibration.style.display = "none"
        # # Set back to default GL splitters
        # self.enable_GL_splitters()

        # Clear the modal body content
        clear_div_modal_body_CameraCalibration()

    def calibrate_n_save_func(self,data):
        print_div("Calibrate & Save button is clicked!<br>")
        loop.create_task(self.async_calibrate_n_save())

    async def async_calibrate_n_save(self):
        try: 
            # Hide cropping modal
            self.modal_CameraCalibration.style.display = "none"
            # # Hide GL splitters
            # self.disable_GL_splitters()

            # Create the file name by looking at the selected camera name at camera feedback layout
            index = self.available_cams_list.selectedIndex
            if index == -1 or len(self.available_cams_list) == 0:
                print_div("No available camera is found or none selected<br>")
                self.camera_name = None
            else:
                self.camera_name = self.camera_node_names_lst[index]
                fileName = self.camera_name + ".yml"

                # Get checkerboard related info from the inputs
                width = int(self.input_width_checkerboard.value)
                height = int(self.input_height_checkerboard.value)
                square_size = float(self.input_square_size.value)

                # Assuming enough pictures are already taken by the user
                # Send these information to camera calibration plugin
                await self.plugin_cameraCalibration.async_calibrate_n_save(fileName, square_size, width, height, None)

                # Update the available calibrated cameras list
                await self.async_update_calibrated_cameras()
                await self.async_select_calibrated_camera()
                

        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
        
    async def async_update_calibrated_cameras(self):
        try:
            # print_div("Clearing the previous available calibrated cameras options..")
            length = self.select_calibrated_cameras.options.length
            i = length-1
            while i >= 0:
                # self.select_calibrated_cameras.options[i] = None
                self.select_calibrated_cameras.remove(i)
                i -= 1

            print_div('Creating available calibrated cameras options..<br>')
            self.calibration_files_lst = await self.plugin_cameraCalibration.async_saved_calibrations(None)
            # print_div(str(self.calibration_files_lst) + "<br>") 
            i = 0
            for fileName in self.calibration_files_lst:
                # Add the available calibration fileName to the select_calibrated_cameras list
                option = document.createElement("option")
                option.text = str(i) + ": " + str(fileName)
                self.select_calibrated_cameras.add(option)
                i += 1 

            # self.rerender_workspace_blocks()
        except:
            import traceback
            print_div(traceback.format_exc())

    def select_calibrated_camera_func(self,data):
        print_div("A Saved Camera Calibration file is selected!<br>")
        self.loop_client.create_task(self.async_select_calibrated_camera())

    async def async_select_calibrated_camera(self):
        # Read the selected calibration file index from the browser
        index = self.select_calibrated_cameras.selectedIndex
        try:
            if index == -1 and len(self.calibration_files_lst) == 0:
                print_div("No available camera calibration file is found<br>")
                # Clear the selected calibration information areas
                print_div_selected_camera_matrix("")
                print_div_selected_camera_distortion_coefficients("")
                print_div_selected_camera_RnT("")

            else:
                if index == -1 and len(self.calibration_files_lst)> 0:
                    print_div("Selecting the first available calibration file")
                    index = 0

                file_name = self.calibration_files_lst[index]
                print_div(file_name)
                params = await self.plugin_cameraCalibration.async_load_calibration(file_name,None)

                
                float_formatter_f = "{:.2f}".format
                float_formatter_e = "{:.2e}".format
                
                np.set_printoptions(formatter={'float_kind':float_formatter_f})
                print_div_selected_camera_matrix(str(params.camera_matrix).replace('\n', '<br> '))

                np.set_printoptions(formatter={'float_kind':float_formatter_e})
                print_div_selected_camera_distortion_coefficients(str(params.distortion_coefficients).replace('\n', '<br> '))

                float_formatter_f = "{:.3f}".format
                np.set_printoptions(formatter={'float_kind':float_formatter_f})
                print_div_selected_camera_RnT(str(params.R_co).replace('\n', '<br> ') + "<br>" + str(params.T_co).replace('\n', '<br> '))

                print_div_selected_camera_calibration_error('{0: 1.3f}'.format(params.error))

                np.set_printoptions(formatter=None)
        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
            pass

    def delete_calibration_func(self,data):
        print_div("Delete selected calibration file button is clicked!<br>")
        self.loop_client.create_task(self.async_delete_calibration())

    async def async_delete_calibration(self):
        # Read the selected calibration file index from the browser
        index = self.select_calibrated_cameras.selectedIndex
        try:
            if index == -1 or len(self.calibration_files_lst) == 0:
                print_div("No available calibration file found or not selected")

            else:
                # if index == -1 and len(self.calibration_files_lst)> 0:
                #     print_div("Selecting the first available calibration file")
                #     index = 0

                file_name = self.calibration_files_lst[index]
                # Ask user: are you sure you want to delete 'file_name'
                resp = window.confirm("Sure you want to delete '" + file_name + "'?")
                # if sure delete, else ignore and return
                if resp:
                    await self.plugin_cameraCalibration.async_delete_calibration(file_name,None)
        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
            pass

        # Update the available calibrated cameras list
        await self.async_update_calibrated_cameras()
        await self.async_select_calibrated_camera()

    def capture_img_calibration_func(self,data):
        print_div("Capturing an image for the calibration..<br>")
        self.loop_client.create_task(self.async_capture_img_calibration())

    async def async_capture_img_calibration(self):
        # Capture the image at the server side of calibration
        await self.plugin_cameraCalibration.async_capture_image(None)

        # Get how many images are taken in the server side of calibration
        num_images = await self.plugin_cameraCalibration.async_num_of_captured_images(None)
        # Print num of images for the user see
        print_div_num_captured_calibration_imgs(str(num_images))


    # Callback functions for ROBOT-CAMERA CALIBRATION 
    # ###############################################################################################################################
    # ###############################################################################################################################
    def calibrate_robotcamera_func(self,data):
        print_div("Calibrate a new robot-camera pair button is pressed.. <br>")
        self.loop_client.create_task(self.async_calibrate_robotcamera())

    async def async_calibrate_robotcamera(self):
        # Ask user: are you sure you want to delete 'file_name'
        resp = window.confirm("Make sure you have defined a safe path in 'Save & Playback' window for robot to travel during calibration. If you defined a safe path, press 'OK'. Otherwise press 'Cancel'.")
        # if sure, else ignore and return
        if resp:
            try: 
                # Show the modal
                self.modal_RobotCameraCalibration.style.display = "block"

                # Connect to: ## Save and Playback Plugin
                print_div('SavePlayback plugin is connecting (from Vision)..<br>')
                self.plugin_savePlayback = await RRN.AsyncConnectService(self.url_plugin_savePlayback,None,None,None,None)
                print_div('SavePlayback plugin is connected (from Vision)!<br>')

                # Remove the previously copied saved poses list element in the page to prevent duplicate
                remove_copied_saved_poses()

                # Clear the calibration serverside captured images
                await self.plugin_cameraRobotCalibration.async_remove_captured_images(None)

                # Show current camera name and current robot name
                # camera name
                index = self.available_cams_list.selectedIndex
                if index == -1 or len(self.available_cams_list) == 0:
                    print_div_robotcamera_calibration_selected_camera("No available camera is found or none selected, SELECT A CAMERA AND TRY AGAIN")
                    self.camera_name = None
                else:
                    self.camera_name = self.camera_node_names_lst[index]
                    print_div_robotcamera_calibration_selected_camera(self.camera_name)

                # robot name
                # ASSUMING ROBOT ALREADY CONNECTED TO SAVEPLAYBACK plugin, get url of the current robot from there
                self.url_robot = str(await self.plugin_savePlayback.async_get_robot_url(None))
                print_div(self.url_robot+ "<br>") # Debug
                # Get all Robot URLs and Robot NodeNames from Discovery Plugin
                self.RobotConnectionURLs = await self.plugin_discovery.async_available_robot_ConnectionURLs(None)
                self.robot_node_names_lst =  await self.plugin_discovery.async_available_robot_NodeNames(None)
                # Create a dict & search for the node name corresponding to url_robot
                self.robot_url_name_dict = dict(zip(self.RobotConnectionURLs,self.robot_node_names_lst))

                self.robot_name = self.robot_url_name_dict.get(self.url_robot)
                if self.robot_name is not None:
                    # Finally Print current robot name
                    print_div_robotcamera_calibration_selected_robot(self.robot_name)
                else:
                    print_div_robotcamera_calibration_selected_robot("No available robot is found or not connected, CONNECT TO A ROBOT AND TRY AGAIN")

                # Copy the saved poses from Save & Playback window
                copy_saved_poses()

            except:
                import traceback
                print_div(traceback.format_exc())
                # raise

    def close_modal_robotcamera_calibration_func(self,data):
        # Hide the modal
        self.modal_RobotCameraCalibration.style.display = "none"

        # Disconnect from: ## Save and Playback Plugin
        print_div('SavePlayback plugin is disconnecting (from Vision)..<br>')
        loop.create_task(self.async_disconnect_from_plugin_savePlayback())
        print_div('SavePlayback plugin is disconnected (from Vision)!<br>')

        # Clear the modal body content
        clear_div_modal_body_RobotCameraCalibration()

    def robotcamera_calibrate_n_save_func(self,data):
        print_div("Robot-Camera Calibrate & Save button is clicked!<br>")
        loop.create_task(self.async_robotcamera_calibrate_n_save())

    async def async_robotcamera_calibrate_n_save(self):
        try: 
            if self.camera_name is not None and self.robot_name is not None:
                # Create the file name by looking at the selected camera name and robot name
                fileName = self.robot_name + "--" + self.camera_name + ".yml"
                print_div(fileName + "<br>") # debug
                print_div(str(type(fileName)) + "<br>") # debug
                
                # Get the ArUco dictionary name from dropdown list
                index = self.select_aruco_marker_dictionary.selectedIndex
                aruco_dict = self.select_aruco_marker_dictionary[index].value
                print_div(aruco_dict+ "<br>") # debug
                print_div(str(type(aruco_dict)) + "<br>") # debug
                
                # Get the aruco id to be detected
                aruco_id = int(self.input_arucomarker_id.value)
                print_div(str(aruco_id)+ "<br>") # debug
                print_div(str(type(aruco_id)) + "<br>") # debug

                # Get the marker side length in millimeters
                aruco_markersize = float(self.input_arucomarker_size.value)
                print_div(str(aruco_markersize)+ "<br>") # debug
                print_div(str(type(aruco_markersize)) + "<br>") # debug

                # Get marker position vector from end effector to marker in end effector frame
                Tx = float(self.input_eef_to_aruco_Tx.value)
                Ty = float(self.input_eef_to_aruco_Ty.value)
                Tz = float(self.input_eef_to_aruco_Tz.value)
                T = np.array(([Tx,Ty,Tz]))
                print_div(str(T)+ "<br>") # debug
                print_div(str(type(T)) + "<br>") # debug

                # Get marker orientation RPY Euler angles in degrees from end effector to marker
                Rx = float(self.input_eef_to_aruco_Rx.value)
                Ry = float(self.input_eef_to_aruco_Ry.value)
                Rz = float(self.input_eef_to_aruco_Rz.value)
                R_rpy = np.array(([Rx,Ry,Rz]))
                print_div(str(R_rpy)+ "<br>") # debug
                print_div(str(type(R_rpy)) + "<br>") # debug

                # Get the camera matrix and distortion coefficients from camera Calibration Plugin (assuming camera intrinsic calibration is already done)
                params = await self.plugin_cameraCalibration.async_load_calibration(self.camera_name + ".yml",None)

                mtx = params.camera_matrix
                print_div(str(mtx)+ "<br>") # debug
                print_div(str(type(mtx)) + "<br>") # debug

                dist = params.distortion_coefficients
                print_div(str(dist)+ "<br>") # debug
                print_div(str(type(dist)) + "<br>") # debug

                # Assuming enough pictures are already taken by the user
                # Send these informations to cameraRobot calibration plugin
                await self.plugin_cameraRobotCalibration.async_calibrate_n_save(fileName, aruco_dict, aruco_id, aruco_markersize, T, R_rpy, mtx, dist, None)

                # Update the available calibrated camera-robot pairs list
                await self.async_update_calibrated_camerarobot_pairs()
                await self.async_select_calibrated_robotcamerapair()

            # Hide cropping modal
            self.modal_RobotCameraCalibration.style.display = "none"   

        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
        
    async def async_update_calibrated_camerarobot_pairs(self):
        try:
            # print_div("Clearing the previous available calibrated camera-robot pair options..")
            length = self.select_calibrated_robotcamerapairs.options.length
            i = length-1
            while i >= 0:
                # self.select_calibrated_robotcamerapairs.options[i] = None
                self.select_calibrated_robotcamerapairs.remove(i)
                i -= 1

            print_div('Creating available calibrated robot-camera pair options..<br>')
            self.robotcamera_calibration_files_lst = await self.plugin_cameraRobotCalibration.async_saved_calibrations(None)
            # print_div(str(self.robotcamera_calibration_files_lst) + "<br>") 
            i = 0
            for fileName in self.robotcamera_calibration_files_lst:
                # Add the available calibration fileName to the select_calibrated_robotcamerapairs list
                option = document.createElement("option")
                option.text = str(i) + ": " + str(fileName)
                self.select_calibrated_robotcamerapairs.add(option)
                i += 1 

            # self.rerender_workspace_blocks()
        except:
            import traceback
            print_div(traceback.format_exc())

    def select_calibrated_robotcamerapair_func(self,data):
        print_div("A Saved Robot-Camera Calibration file is selected!<br>")
        self.loop_client.create_task(self.async_select_calibrated_robotcamerapair())

    async def async_select_calibrated_robotcamerapair(self):
        # Read the selected calibration file index from the browser
        index = self.select_calibrated_robotcamerapairs.selectedIndex
        try:
            if index == -1 and len(self.robotcamera_calibration_files_lst) == 0:
                print_div("No available robot-camera calibration file is found<br>")
                # Clear the selected robot camera calibration information areas
                print_div_selected_robotcamerapair_rotation_matrix("")
                print_div_selected_robotcamerapair_translation_vector("")

            else:
                if index == -1 and len(self.robotcamera_calibration_files_lst)> 0:
                    print_div("Selecting the first available calibration file")
                    index = 0

                file_name = self.robotcamera_calibration_files_lst[index]
                print_div(file_name)
                params = await self.plugin_cameraRobotCalibration.async_load_calibration(file_name,None)

                float_formatter_f = "{:.3f}".format
                np.set_printoptions(formatter={'float_kind':float_formatter_f})
                print_div_selected_robotcamerapair_rotation_matrix(str(params.R_oc).replace('\n', '<br> ')) 
                print_div_selected_robotcamerapair_translation_vector(str(params.T_oc).replace('\n', '<br> '))

                np.set_printoptions(formatter=None)
        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
            pass

    def delete_robotcamerapair_calibration_func(self,data):
        print_div("Delete selected robot-camera calibration file button is clicked!<br>")
        self.loop_client.create_task(self.async_delete_robotcamerapair_calibration())

    async def async_delete_robotcamerapair_calibration(self):
        # Read the selected calibration file index from the browser
        index = self.select_calibrated_robotcamerapairs.selectedIndex
        try:
            if index == -1 or len(self.robotcamera_calibration_files_lst) == 0:
                print_div("No available robot-camera calibration file found or not selected")

            else:
                # if index == -1 and len(self.robotcamera_calibration_files_lst)> 0:
                #     print_div("Selecting the first available robot-camera calibration pair file")
                #     index = 0

                file_name = self.robotcamera_calibration_files_lst[index]
                # Ask user: are you sure you want to delete 'file_name'
                resp = window.confirm("Sure you want to delete '" + file_name + "'?")
                # if sure delete, else ignore and return
                if resp:
                    await self.plugin_cameraRobotCalibration.async_delete_calibration(file_name,None)
        except:
            import traceback
            print_div(traceback.format_exc())
            # raise
            pass

        # Update the available calibrated cameras list
        await self.async_update_calibrated_camerarobot_pairs()
        await self.async_select_calibrated_robotcamerapair()

    def jog_sel_pose_func(self,data):
        print_div("Jogging to the selected pose..<br>")
        self.loop_client.create_task(self.async_jog_sel_pose())

    async def async_jog_sel_pose(self):
        # Read the selected pose index from the browser
        element_id = "saved_poses_list_clone"
        poses_list = document.getElementById(element_id)
        index = poses_list.selectedIndex
        print_div("selected index: " + str(index) + "<br>") # debug
        try:
            if index == -1:
                print_div("Please select a pose from Saved Poses.<br>")
            else:
                await self.plugin_savePlayback.async_go_sel_pose(index,None)
        except:
            pass

    def jog_prev_pose_func(self,data):
        print_div("Jogging to previous pose..<br>")
        self.loop_client.create_task(self.async_jog_prev_pose())

    async def async_jog_prev_pose(self):
        # Read the selected pose index from the browser
        element_id = "saved_poses_list_clone"
        poses_list = document.getElementById(element_id)
        if poses_list.length > 0:
            index = poses_list.selectedIndex - 1
            print_div("selected index: " + str(index) + "<br>") # debug
            try:
                if index == -1:
                    index = 0 # select the first saved pose

                poses_list.selectedIndex = index # on UI also select the previous pose
                await self.plugin_savePlayback.async_go_sel_pose(index,None)
            except:
                import traceback
                print_div(traceback.format_exc())
                pass
        else:
            print_div("There is no saved pose. Save some poses and try again<br>")


    def jog_next_pose_func(self,data):
        print_div("Jogging to next pose..<br>")
        self.loop_client.create_task(self.async_jog_next_pose())

    async def async_jog_next_pose(self):
        # Read the selected pose index from the browser
        element_id = "saved_poses_list_clone"
        poses_list = document.getElementById(element_id)
        if poses_list.length > 0:
            index = poses_list.selectedIndex + 1
            print_div("selected index: " + str(index) + "<br>") # debug
            try:
                if index >= poses_list.length:
                    index = poses_list.length-1 # select the last saved pose

                poses_list.selectedIndex = index # on UI also select the next pose
                await self.plugin_savePlayback.async_go_sel_pose(index,None)
            except:
                import traceback
                print_div(traceback.format_exc())
                pass
        else:
            print_div("There is no saved pose. Save some poses and try again<br>")

    def capture_img_robotcamera_calibration_func(self,data):
        print_div("Capturing an image for the robot-camera calibration..<br>")
        self.loop_client.create_task(self.async_capture_img_robotcamera_calibration())

    async def async_capture_img_robotcamera_calibration(self):
        try:
            # Get current robot pose from save & playback plugin
            current_robot_pose = await self.plugin_savePlayback.async_current_robot_pose(None)

            print_div(str(current_robot_pose.R) + "<br>" + str(current_robot_pose.T))

            # Capture the image at the server side of calibration with the current robot pose
            await self.plugin_cameraRobotCalibration.async_capture_image_with_robot_pose(current_robot_pose.R,current_robot_pose.T,None)

            # Get how many images are taken in the server side of calibration
            num_images = await self.plugin_cameraRobotCalibration.async_num_of_captured_images(None)
            # Print num of images for the user see
            print_div_num_captured_robotcamera_calibration_imgs(str(num_images))
        except:
            import traceback
            print_div(traceback.format_exc())
            pass


    def stop_robot_func(self,data):
        print_div("Stopping the robot..<br>")
        self.loop_client.create_task(self.async_stop_robot())

    async def async_stop_robot(self):
        # Capture the image at the server side of calibration
        await self.plugin_savePlayback.async_stop_joints(None)
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
    
    # ip_plugins = '128.113.224.98'
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

loop = asyncio.get_event_loop()
loop.create_task(client_vision())

# RRN.PostToThreadPool(client_vision()) 