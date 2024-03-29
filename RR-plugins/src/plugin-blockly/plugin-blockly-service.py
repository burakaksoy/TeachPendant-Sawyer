import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np
import time

import general_robotics_toolbox as rox #mainly used to define the pose(position-orientaion) related block functionalities

import os
from os import listdir
from os.path import isfile, join

import math

# import glob # To get the xml file names
# if not os.path.exists('my_folder'):
#     os.makedirs('my_folder')
# print(glob.glob("/home/adam/*.txt"))

class Blockly_impl(object):
    def __init__(self):
        # Active robot related info
        self.url_robot = None
        self.RobotConnectionURLs = None
        self.robot_node_names_lst = None
        self.robot_url_name_dict = None
        self.robot_name = None

        self.url_plugins_lst = [] # The order will be :
        # url_plugin_discovery, url_plugin_jogJointSpace, url_plugin_jogCartesianSpace, url_plugin_savePlayback
        self.plugin_discovery = None
        self.plugin_jogJointSpace = None
        self.plugin_jogCartesianSpace = None
        self.plugin_savePlayback = None
        self.plugin_tool = None
        self.plugin_toolLinkAttacher = None

        self.url_plugins_vision_lst = [] # The order will be :
        # url_plugin_cameraFeedback, url_plugin_cameraTraining, url_plugin_cameraCalibration, url_plugin_cameraTracking
        self.plugin_cameraFeedback = None
        self.plugin_cameraTraining = None
        self.plugin_cameraCalibration = None
        self.plugin_cameraTracking = None
        self.plugin_cameraRobotCalibration = None

        # Create a folder for saved blockly workspaces
        self.path = "./blockly-savedWorkspaces"
        self.extension = ".xml"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

        self.blockly_saved_workspaces()

    def reset(self):
        self.url_robot = None
        self.RobotConnectionURLs = None
        self.robot_node_names_lst = None
        self.robot_url_name_dict = None
        self.robot_name = None

        self.url_plugins_lst = []

        self.plugin_discovery = None
        self.plugin_jogJointSpace = None
        self.plugin_jogCartesianSpace = None
        self.plugin_savePlayback = None
        self.plugin_tool = None
        self.plugin_toolLinkAttacher = None

        # Create a folder for saved blockly workspaces
        self.path = "./blockly-savedWorkspaces"
        self.extension = ".xml"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

        self.blockly_saved_workspaces()

    def reset_vision(self):
        self.url_plugins_vision_lst = [] 
        
        self.plugin_cameraFeedback = None
        self.plugin_cameraTraining = None
        self.plugin_cameraCalibration = None
        self.plugin_cameraTracking = None
        self.plugin_cameraRobotCalibration = None

        # Create a folder for saved blockly workspaces
        self.path = "./blockly-savedWorkspaces"
        self.extension = ".xml"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

        self.blockly_saved_workspaces()

    def connect2plugins(self, url_plugins_lst):
        if not self.url_plugins_lst: # if the list is empty
            self.url_plugins_lst = url_plugins_lst # append the new urls
            # self.url_plugins_lst = list(set(self.url_plugins_lst)) # keep only the unique urls, prevent adding the same urls again
            print("robot plugin urls:")
            print(self.url_plugins_lst)

            if not self.url_plugins_lst[0] == '':
                self.plugin_discovery = RRN.ConnectService(self.url_plugins_lst[0])
            if not self.url_plugins_lst[1] == '':
                self.plugin_jogJointSpace = RRN.ConnectService(self.url_plugins_lst[1])
            if not self.url_plugins_lst[2] == '':
                self.plugin_jogCartesianSpace = RRN.ConnectService(self.url_plugins_lst[2])
            if not self.url_plugins_lst[3] == '':
                self.plugin_savePlayback = RRN.ConnectService(self.url_plugins_lst[3])
                self.find_active_robot_name()
            if not self.url_plugins_lst[4] == '':
                self.plugin_tool = RRN.ConnectService(self.url_plugins_lst[4])
            if not self.url_plugins_lst[5] == '':
                self.plugin_toolLinkAttacher = RRN.ConnectService(self.url_plugins_lst[5])
        else:
            # Give an error that says the robot plugins are already connected
            print("Robot plugins are already connected to Blockly service! Trying to connect again..")
            self.reset()
            self.connect2plugins(url_plugins_lst)

    def connect2plugins_vision(self, url_plugins_vision_lst):
        if not self.url_plugins_vision_lst: # if the list is empty
            self.url_plugins_vision_lst = url_plugins_vision_lst # append the new urls
            # self.url_plugins_vision_lst = list(set(self.url_plugins_vision_lst)) # keep only the unique urls, prevent adding the same urls again
            print("vision plugin urls:")
            print(self.url_plugins_vision_lst)

            if not self.url_plugins_vision_lst[0] == '':
                self.plugin_cameraFeedback = RRN.ConnectService(self.url_plugins_vision_lst[0])
            if not self.url_plugins_vision_lst[1] == '':
                self.plugin_cameraTraining = RRN.ConnectService(self.url_plugins_vision_lst[1])
            if not self.url_plugins_vision_lst[2] == '':
                self.plugin_cameraCalibration = RRN.ConnectService(self.url_plugins_vision_lst[2])
            if not self.url_plugins_vision_lst[3] == '':
                self.plugin_cameraTracking = RRN.ConnectService(self.url_plugins_vision_lst[3])
            if not self.url_plugins_vision_lst[4] == '':
                self.plugin_cameraRobotCalibration = RRN.ConnectService(self.url_plugins_vision_lst[4])

        else:
            # Give an error that says the robot plugins are already connected
            print("Vision plugins are already connected to Blockly service! Trying to connect again..")
            self.reset_vision()
            self.connect2plugins_vision(url_plugins_vision_lst)

    def find_active_robot_name(self):
        # robot name
        # ASSUMING ROBOT ALREADY CONNECTED TO SAVEPLAYBACK plugin, get url of the current robot from there
        self.url_robot = str(self.plugin_savePlayback.robot_url)
        print(self.url_robot+ " is the url_robot") # Debug
        # Get all Robot URLs and Robot NodeNames from Discovery Plugin
        self.RobotConnectionURLs = self.plugin_discovery.available_robot_ConnectionURLs()
        self.robot_node_names_lst = self.plugin_discovery.available_robot_NodeNames()
        # Create a dict & search for the node name corresponding to url_robot
        self.robot_url_name_dict = dict(zip(self.RobotConnectionURLs,self.robot_node_names_lst))

        self.robot_name = self.robot_url_name_dict.get(self.url_robot)
        if self.robot_name is not None:
            # Finally Print current robot name
            print(self.robot_name)
        else:
            print("No available robot is found or not connected, CONNECT TO A ROBOT AND TRY AGAIN")

    def blockly_saved_workspaces(self):
        # self.saved_workspaces_filenames = [f for f in listdir(self.path) if isfile(join(self.path, f))]
        self.saved_workspaces_filenames = [f for f in listdir(self.path) if f.endswith(self.extension)]

        print(self.saved_workspaces_filenames)
        return self.saved_workspaces_filenames


    def blockly_load_workspace(self, filename):
        with open( self.path +"/"+ filename, 'r') as file:
            xml_str = file.read().replace('\n', '')

        return xml_str

    def blockly_delete_workspace(self,filename):
        if os.path.exists(self.path +"/"+ filename):
            os.remove(self.path +"/"+ filename)
        else:
            print("The file does not exist")

    def blockly_edit_workspace_name(self, filename, file_name_new):
        if os.path.exists(self.path +"/"+ filename):
            os.rename(self.path +"/"+ filename, self.path +"/"+ file_name_new) 
            # TODO: Check the existance of the new file name, maybe
        else:
            print("The file does not exist")

    def blockly_save_workspace(self, filename, workspace_xml):
        if os.path.exists(self.path +"/"+ filename):
            with open(self.path +"/"+ filename, 'w') as filetowrite:
                filetowrite.write(workspace_xml)
        else:
            print("The file does not exist")

    def blockly_save_workspace_as(self, filename, workspace_xml):
        if not os.path.exists(self.path +"/"+ filename):
            with open(self.path +"/"+ filename, 'w') as filetowrite:
                filetowrite.write(workspace_xml)
        else:
            print("The file already exists!")

    # Blockly Code Execution
    def execute_blockly(self, code_text):
        output = code_text

        exec(code_text)

        return output

    # implementations of blockly functions

    # --- ROBOT related implementation of blockly functions: BEGIN -----------
    def jog_joint(self, dropdown_joint_selected ,value_degree, speed_perc):
        try:
            self.plugin_jogJointSpace.jog_joint_to_angle(dropdown_joint_selected-1, np.deg2rad(float(value_degree)), float(speed_perc))
        except:
            import traceback
            print(traceback.format_exc())

        
    def jog_joint_relative(self, dropdown_joint_selected ,value_degree, speed_perc):
        try:
            joint_angles = np.zeros((7,))
            joint_angles[dropdown_joint_selected-1] = np.deg2rad(float(value_degree))
            self.plugin_jogJointSpace.jog_joints_to_angles_relative(joint_angles, float(speed_perc))
        except:
            import traceback
            print(traceback.format_exc())


    def jog_joints(self, degrees_lst, speed_perc):
        try:
            joint_angles = np.deg2rad(np.asarray(degrees_lst, dtype=np.float))
            self.plugin_jogJointSpace.jog_joints_to_angles2(joint_angles, float(speed_perc))
        except:
            import traceback
            print(traceback.format_exc())

        
    def jog_joints_relative(self, degrees_lst, speed_perc):
        try:
            joint_angles = np.deg2rad(np.asarray(degrees_lst, dtype=np.float))
            self.plugin_jogJointSpace.jog_joints_to_angles_relative(joint_angles, float(speed_perc))
        except:
            import traceback
            print(traceback.format_exc())

    def jog_cartesian(self, P_lst, R_angles_lst, speed_perc):
        try:
            # self.plugin_jogCartesianSpace.prepare_jog()
            
            P = np.asarray(P_lst, dtype=np.float)
            R_angles = np.asarray(R_angles_lst, dtype=np.float)
            R_angles = np.deg2rad(R_angles) # Convert deg to rad 

            self.plugin_jogCartesianSpace.jog_cartesian_with_speed(P,R_angles,speed_perc)
        except:
            import traceback
            print(traceback.format_exc())    

    def jog_cartesian_relative(self, P_lst, R_angles_lst, speed_perc):
        try:
            # self.plugin_jogCartesianSpace.prepare_jog()
            
            P = np.asarray(P_lst, dtype=np.float)
            R_angles = np.asarray(R_angles_lst, dtype=np.float)
            R_angles = np.deg2rad(R_angles) # Convert deg to rad 

            self.plugin_jogCartesianSpace.jog_cartesian_relative_with_speed(P,R_angles,speed_perc)
        except:
            import traceback
            print(traceback.format_exc())   
    # --- ROBOT related implementation of blockly functions: END -----------   

    # --- VISION related implementation of blockly functions: BEGIN -----------

    def camera_get_object_pose_z_required(self, dropdown_trained_objects, dropdown_cams, value_z_distance ):
        try:
            # # Execute the object detection in image frame
            # return_result_image = False
            # detection_result = self.plugin_cameraTracking.find_object_in_img_frame(dropdown_trained_objects, dropdown_cams, return_result_image)

            print("")
            print("------------------------------")
            print("Camera: " + str(dropdown_cams) + ", Object: " + str(dropdown_trained_objects))
            # print("Detected Object Size(w,h): ("+ str(detection_result.width) + "," + str(detection_result.height) + ")")
            # print("Detected Object Center(x,y): ("+ str(detection_result.center_x) + "," + str(detection_result.center_y)+ ")")
            # print("Detected Object Angle: " + str(detection_result.angle) + " degrees")

            pose = self.plugin_cameraTracking.find_object_pose_in_cam_frame(dropdown_trained_objects, dropdown_cams, value_z_distance)

            print("R: " + str(pose.R))
            print("T: " + str(pose.T))

            return pose
        except:
            import traceback
            print(traceback.format_exc())  


    def camera_get_object_pose_z_not_required(self, dropdown_trained_objects, dropdown_cams):
        try:
            # TODO
            pose = self.plugin_cameraTracking.find_object_pose_in_cam_frame(dropdown_trained_objects, dropdown_cams, 0)

            print("R: " + str(pose.R))
            print("T: " + str(pose.T))

            return pose
        except:
            import traceback
            print(traceback.format_exc()) 

    def camera_transform_pose_to_robot(self, value_pose_in_cam, dropdown_cams):
        try:
            # A pose is given in the camera frame.
            # Find the corresponding pose in the robot base frame.   
            Rc_obj = value_pose_in_cam.R
            Tc_obj = value_pose_in_cam.T

            # Pose of the camera wrt robot base

            # # T0_c = np.asarray([0.5,0,0.73], dtype=np.float32) # Predefined when camera is fixed
            # T0_c = np.asarray([0.6659,-0.8237,1.1862], dtype=np.float32) # Predefined when camera is fixed (LAB KINECT)

            # # R0_c = np.asarray([[0,1,0],[1,0,0],[0,0,-1]], dtype=np.float32) # Predefined when camera is fixed
            # R0_c = np.asarray([[0,-1,0],[-1,0,0],[0,0,-1]], dtype=np.float32) # Predefined when camera is fixed (LAB KINECT)

            filename = self.robot_name + "--" + dropdown_cams + ".yml"
            params = self.plugin_cameraRobotCalibration.load_calibration(filename)
            T0_c = params.T_oc/1000.0 # convert to meters
            R0_c = params.R_oc

            # To grab the object via end effector, the orientation btw them should be:
            RE_obj = np.asarray([[0,0,-1],[-1,0,0],[0,1,0]], dtype=np.float32)

            # We need to find 
            T0_obj = (R0_c @ Tc_obj) + T0_c # Tranlastion from base frame to object origin in base frame
            R0_E = R0_c @ Rc_obj @ RE_obj.T # Rotation between the initial end eff. and the desired end effector

            # Return the same type pose with the new calculated values
            value_pose_in_cam.R = R0_E
            
            # position_offset = np.asarray([0,0,0.1], dtype=np.float32) # TODO: THIS HAS TO BE REMOVED AND HANDLED WITH THE POSE RELATED ARITHMATIC BLOCKS, THIS IS JUST TEMPORARY!!!
            # value_pose_in_cam.T = T0_obj + position_offset
            value_pose_in_cam.T = T0_obj

            print("value_pose_in_cam.R" + str(value_pose_in_cam.R))
            print("value_pose_in_cam.T" + str(value_pose_in_cam.T))

            return value_pose_in_cam

        except:
            import traceback
            print(traceback.format_exc()) 

    # --- VISION related implementation of blockly functions: END -----------   

    # --- UTILS related implementation of blockly functions: BEGIN -----------        
    def utils_position_in_pose(self,value_pose):
        try:
            T = value_pose.T # T is a numpy vector
            # We need to convert it to list 
            print("T: "+ str(T.tolist()))
            return T.tolist()
        except:
            import traceback
            print(traceback.format_exc()) 

    #########################################################
    # ZYX Euler angles calculation from rotation matrix
    def isclose(self,x, y, rtol=1.e-5, atol=1.e-8):
        return abs(x-y) <= atol + rtol * abs(y)

    def euler_angles_from_rotation_matrix(self,R):
        '''
        From a paper by Gregory G. Slabaugh (undated),
        "Computing Euler angles from a rotation matrix
        '''
        phi = 0.0
        if self.isclose(R[2,0],-1.0):
            theta = math.pi/2.0
            psi = math.atan2(R[0,1],R[0,2])
        elif self.isclose(R[2,0],1.0):
            theta = -math.pi/2.0
            psi = math.atan2(-R[0,1],-R[0,2])
        else:
            theta = -math.asin(R[2,0])
            cos_theta = math.cos(theta)
            psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
            phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
        return psi, theta, phi #  x y z for Rz * Ry * Rx
    #########################################################

    def utils_orientation_in_pose(self,value_pose):
        try:
            R = value_pose.R # R is a 3x3 numpy array
            # Calculate euler ZYX angles from Rotation matrix
            x,y,z = self.euler_angles_from_rotation_matrix(R)
            # Convert from radian to degree
            R_angles = np.rad2deg([x,y,z]) # Convert rad to deg 
            # We need to convert it to list
            print("R: "+ str(R_angles.tolist())) 
            return R_angles.tolist()

        except:
            import traceback
            print(traceback.format_exc())

    def utils_rotate_vector(self, value_orientation,value_position):
        try:
            rpy = np.asarray(value_orientation) # convert to numpy array from list
            rpy = np.deg2rad(rpy) # convert from deg to radian
            R = rox.rpy2R(rpy)# calculate corresponding rotation matrix

            T = R @ np.asarray(value_position) # execute the rotation of the vector
            
            print("T(utils_inverse_frame): "+ str(T.tolist())) 
            return T.tolist() # return as list 
        except:
            import traceback
            print(traceback.format_exc())

    def utils_rotate_frame(self, value_orientation_1,value_orientation_2):
        try:
            rpy = np.asarray(value_orientation_1) # convert to numpy array from list
            rpy = np.deg2rad(rpy) # convert from deg to radian
            R1 = rox.rpy2R(rpy)# calculate corresponding rotation matrix

            rpy = np.asarray(value_orientation_2) # convert to numpy array from list
            rpy = np.deg2rad(rpy) # convert from deg to radian
            R2 = rox.rpy2R(rpy)# calculate corresponding rotation matrix

            R = R1 @ R2 # execute the rotation

            x,y,z = self.euler_angles_from_rotation_matrix(R) # convert back to RPY euler angles from rotation matrix
            R_angles = np.rad2deg([x,y,z]) # convert back to degrees from radian
            print("R(utils_rotate_frame): "+ str(R_angles.tolist())) 
            return R_angles.tolist() # return as list 
        except:
            import traceback
            print(traceback.format_exc())

    def utils_sum_vectors(self, value_position_1,value_position_2):
        try:
            return (np.asarray(value_position_1)+np.asarray(value_position_2)).tolist()
        except:
            import traceback
            print(traceback.format_exc())

    def utils_subtract_vectors(self, value_position_1,value_position_2):
        try:
            return (np.asarray(value_position_1)-np.asarray(value_position_2)).tolist()
        except:
            import traceback
            print(traceback.format_exc())

    def utils_inverse_frame(self, value_orientation):
        try:
            rpy = np.asarray(value_orientation) # convert to numpy array from list
            rpy = np.deg2rad(rpy) # convert from deg to radian
            R = rox.rpy2R(rpy)# calculate corresponding rotation matrix
            R = R.T # calculate the inverse i.e. transpose
            x,y,z = self.euler_angles_from_rotation_matrix(R) # convert back to RPY euler angles from rotation matrix
            R_angles = np.rad2deg([x,y,z]) # convert back to degrees from radian
            print("R(utils_inverse_frame): "+ str(R_angles.tolist())) 
            return R_angles.tolist() # return as list 
        except:
            import traceback
            print(traceback.format_exc())

    def utils_minus_vector(self, value_position):
        try:
            return (-np.asarray(value_position)).tolist()
        except:
            import traceback
            print(traceback.format_exc())


    def utils_pose_from_position_n_orientation(self, value_position, value_orientation):
        try:
            rpy = np.asarray(value_orientation) # convert to numpy array from list
            rpy = np.deg2rad(rpy) # convert from deg to radian
            R = rox.rpy2R(rpy)# calculate corresponding rotation matrix
            T = R @ np.asarray(value_position)
            
            # Get the RR pose structure
            pose = RRN.NewStructure("experimental.pluginBlockly.Pose")
            pose.R = R
            pose.T = T

            return pose
        except:
            import traceback
            print(traceback.format_exc())

    # --- UTILS related implementation of blockly functions: END -----------        

    # --- TOOLS related implementation of blockly functions: BEGIN -----------        
    def tool_link_attacher(self,text_robot_name,text_obj_name,dropdown_status):
        try:
            # todo
            self.plugin_toolLinkAttacher.attach_link(str(text_robot_name),str(text_obj_name), int(dropdown_status))
        except:
            import traceback
            print(traceback.format_exc())

    def tool_gripper(self,dropdown_status):
        try:
            if bool(int(dropdown_status)):
                self.plugin_tool.active_tool_open()
            else:
                self.plugin_tool.active_tool_close()
        except:
            import traceback
            print(traceback.format_exc())     
    # --- TOOLS related implementation of blockly functions: END -----------


def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-blockly-service", 8897) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginBlockly")

        # create object
        Blockly_inst = Blockly_impl()
        # register service with service name "Blockly", type "experimental.pluginBlockly.Blockly", actual object: Blockly_inst
        RRN.RegisterService("Blockly","experimental.pluginBlockly.Blockly",Blockly_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginBlockly Server started, press enter to quit...")
        else:
            raw_input("pluginBlockly Server started, press enter to quit...")

if __name__ == '__main__':
    main()