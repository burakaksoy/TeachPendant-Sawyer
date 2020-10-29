import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np
import time

class Blockly_impl(object):
    def __init__(self):
        self.url_plugins_lst = [] # The orher will be :
        # url_plugin_jogJointSpace, url_plugin_jogCartesianSpace, url_plugin_savePlayback

        self.plugin_jogJointSpace = None
        self.plugin_jogCartesianSpace = None
        self.plugin_savePlayback = None

    def reset(self):
        self.url_plugins_lst = []

        self.plugin_jogJointSpace = None
        self.plugin_jogCartesianSpace = None
        self.plugin_savePlayback = None


    def connect2plugins(self, url_plugins_lst):
        if not self.url_plugins_lst: # if the list is empty
            self.url_plugins_lst = url_plugins_lst # append the new urls
            # self.url_plugins_lst = list(set(self.url_plugins_lst)) # keep only the unique urls, prevent adding the same urls again
            print(self.url_plugins_lst)

            self.plugin_jogJointSpace = RRN.ConnectService(self.url_plugins_lst[0])
            self.plugin_jogCartesianSpace = RRN.ConnectService(self.url_plugins_lst[1])
            self.plugin_savePlayback = RRN.ConnectService(self.url_plugins_lst[2])
            

        else:
            # Give an error that says the robot plugins are already connected
            print("Robot plugins are already connected to Blockly service! Trying to connect again..")
            self.reset()
            self.connect2plugins(url_plugins_lst)

    def execute_blockly(self, code_text):
        output = code_text

        exec(code_text)

        return output

    # implementations of blockly functions
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