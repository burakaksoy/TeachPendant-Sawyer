import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np
import time


import os
from os import listdir
from os.path import isfile, join

# import glob # To get the xml file names
# if not os.path.exists('my_folder'):
#     os.makedirs('my_folder')
# print(glob.glob("/home/adam/*.txt"))

class Blockly_impl(object):
    def __init__(self):
        self.url_plugins_lst = [] # The order will be :
        # url_plugin_jogJointSpace, url_plugin_jogCartesianSpace, url_plugin_savePlayback
        self.plugin_jogJointSpace = None
        self.plugin_jogCartesianSpace = None
        self.plugin_savePlayback = None

        self.url_plugins_vision_lst = [] # The order will be :
        # url_plugin_cameraFeedback, url_plugin_cameraTraining, url_plugin_cameraCalibration, url_plugin_cameraTracking
        self.plugin_cameraFeedback = None
        self.plugin_cameraTraining = None
        self.plugin_cameraCalibration = None
        self.plugin_cameraTracking = None

        # Create a folder for saved blockly workspaces
        self.path = "./blockly-savedWorkspaces"
        self.extension = ".xml"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

        self.blockly_saved_workspaces()

    def reset(self):
        self.url_plugins_lst = []

        self.plugin_jogJointSpace = None
        self.plugin_jogCartesianSpace = None
        self.plugin_savePlayback = None

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

            self.plugin_jogJointSpace = RRN.ConnectService(self.url_plugins_lst[0])
            self.plugin_jogCartesianSpace = RRN.ConnectService(self.url_plugins_lst[1])
            self.plugin_savePlayback = RRN.ConnectService(self.url_plugins_lst[2])
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

            self.plugin_cameraFeedback = RRN.ConnectService(self.url_plugins_vision_lst[0])
            self.plugin_cameraTraining = RRN.ConnectService(self.url_plugins_vision_lst[1])
            self.plugin_cameraCalibration = RRN.ConnectService(self.url_plugins_vision_lst[2])
            self.plugin_cameraTracking = RRN.ConnectService(self.url_plugins_vision_lst[3])

        else:
            # Give an error that says the robot plugins are already connected
            print("Vision plugins are already connected to Blockly service! Trying to connect again..")
            self.reset_vision()
            self.connect2plugins_vision(url_plugins_vision_lst)

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
            self.plugin_jogCartesianSpace.prepare_jog()
            
            P = np.asarray(P_lst, dtype=np.float)
            R_angles = np.asarray(R_angles_lst, dtype=np.float)
            R_angles = np.deg2rad(R_angles) # Convert deg to rad 

            self.plugin_jogCartesianSpace.jog_cartesian_with_speed(P,R_angles,speed_perc)
        except:
            import traceback
            print(traceback.format_exc())    

    def jog_cartesian_relative(self, P_lst, R_angles_lst, speed_perc):
        try:
            self.plugin_jogCartesianSpace.prepare_jog()
            
            P = np.asarray(P_lst, dtype=np.float)
            R_angles = np.asarray(R_angles_lst, dtype=np.float)
            R_angles = np.deg2rad(R_angles) # Convert deg to rad 

            self.plugin_jogCartesianSpace.jog_cartesian_relative_with_speed(P,R_angles,speed_perc)
        except:
            import traceback
            print(traceback.format_exc())   
    # --- ROBOT related implementation of blockly functions: END -----------   

    # --- VISION related implementation of blockly functions: BEGIN -----------

    # --- VISION related implementation of blockly functions: END -----------   

        



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