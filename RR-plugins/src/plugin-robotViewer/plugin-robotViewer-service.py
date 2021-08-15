import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np
import time

import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.IdentifierUtil import IdentifierUtil



class RobotViewer_impl(object):
    def __init__(self):
        self.url_robot = None
        self.robot = None ## RR robot object
        self.is_robot_connected = False


    def reset(self):
        self.url_robot = None
        self.robot = None ## RR robot object
        self.is_robot_connected = False
    
    def connect2robot(self, url_robot):
        if self.robot is None:
            self.url_robot = url_robot

            # self.robot = RRN.ConnectService(self.url_robot) # connect to robot with the given url
            self.robot_sub = RRN.SubscribeService(self.url_robot)
            self.robot = self.robot_sub.GetDefaultClientWait(1) 
            
            self.is_robot_connected = True

            try:
                # Connect to tesseract viewer service
                self.tesseract_viewer = RRN.ConnectService('rr+tcp://localhost:59712?service=tesseract_viewer')

                # Get the joint names in the tesseract viewer
                print(self.tesseract_viewer.joint_names)

                # Sawyer
                self.joint_names = [f"right_j{i}" for i in range(7)] # Note: "head_pan" joint is not included 
                # # ABB
                # self.joint_names = [f"ABB1200_joint_{i+1}" for i in range(6)]
                # # UR5
                # self.joint_names = ["shoulder_pan_joint","shoulder_lift_joint", "elbow_joint", "wrist_1_joint","wrist_2_joint","wrist_3_joint"]

            except:
                import traceback
                print(traceback.format_exc())
                print("Could not connect to tesseract viewer service")


            # log that the robot is successfully connected  
            print("Robot is connected to plugin robotViewer service!")
        else:
            # Give an error that says the robot is already connected
            print("Robot is already connected to plugin robotViewer service! Trying to connect again..")
            self.reset()
            self.connect2robot(url_robot)

    def update_current_view(self):
        if self.is_robot_connected:
            try:
                # get the current joint angles from the robot
                cur_q = self.get_current_joint_positions()
                # Send the joint angles to tesseract viewer service to update
                self.tesseract_viewer.update_joint_positions(self.joint_names, cur_q)
            except:
                import traceback
                print(traceback.format_exc())
                print("Could not update the tesseract viewer joint angles")

        else:
            # Give an error message to show that the robot is not connected
            print("update_current_view failed. robot is not connected to RobotViewer service yet!")

    def get_current_joint_positions(self):
        cur_robot_state = self.robot.robot_state.PeekInValue()    
        cur_q = cur_robot_state[0].joint_position
        return cur_q # in radian ndarray

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-robotViewer-service", 8901) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginRobotViewer")
        RRC.RegisterStdRobDefServiceTypes(RRN)

        # create object
        RobotViewer_inst = RobotViewer_impl()
        # register service with service name "RobotViewer", type "experimental.pluginRobotViewer.RobotViewer", actual object: RobotViewer_inst
        RRN.RegisterService("RobotViewer","experimental.pluginRobotViewer.RobotViewer",RobotViewer_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginRobotViewer Server started, press enter to quit...")
        else:
            raw_input("pluginRobotViewer Server started, press enter to quit...")

if __name__ == '__main__':
    main()