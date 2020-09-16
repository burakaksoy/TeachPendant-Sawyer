import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect to robot 
import numpy as np

class JogCartesianSpace_impl(object):
    def __init__(self):
        self.url_robot = None
        self.robot = None

        # Incremental difference amounts to jog in cartesian space
        self.move_distance = 0.01 # meters
        self.rotate_angle = np.deg2rad(5) # radians

    def jog_cartesian(self, P_axis, R_axis):
        print("Jog Joints is called")

        if self.robot is not None:
            # Put the robot to jogging mode
            self.robot.command_mode = self.halt_mode
            # time.sleep(0.1)
            self.robot.command_mode = self.jog_mode
            # time.sleep(0.1)

            # Jog the robot in cartesian space
            
            # # get the current joint angles

            # cur_q = self.get_current_joint_positions()

            print("Jog in Cartesian Space with command P_axis" + str(P_axis) + "and R_axis"+ str(R_axis))
            


        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def connect2robot(self, url_robot):
        if self.robot is None:
            self.url_robot = url_robot
            self.robot = RRN.ConnectService(self.url_robot) # connect to robot with the given url
            
            # self.robot.reset_errors()
            # self.robot.enable()

            # Define Robot modes
            self.robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", self.robot)
            self.halt_mode = self.robot_const["RobotCommandMode"]["halt"]
            self.jog_mode = self.robot_const["RobotCommandMode"]["jog"]
            
            # self.position_mode = self.robot_const["RobotCommandMode"]["velocity_command"]
            # self.trajectory_mode = self.robot_const["RobotCommandMode"]["trajectory"]

            self.assign_robot_details()
            
            # log that the robot is successfully connected  
            print("Robot is connected to JogCartesianSpace service!")
        else:
            # Give an error that says the robot is already connected
            print("Robot is already connected to JogCartesianSpace service! Trying to connect again..")
            self.robot = None
            self.connect2robot(url_robot)

    def assign_robot_details(self):
        if self.robot is not None:
            self.robot_info = self.robot.robot_info
            self.joint_info = self.robot_info.joint_info # A list of jointInfo

            self.joint_types = [] # A list or array of N numbers containing the joint type. 1 for rotary, 3 for prismatic
            self.joint_lower_limits = [] # list or numpy.array
            self.joint_upper_limits = [] # list or numpy.array
            self.joint_vel_limits = [] # list or numpy.array
            self.joint_acc_limits = [] # list or numpy.array
            self.joint_names = [] # list of string
            self.joint_uuids = [] 
            for joint in self.joint_info:
                self.joint_types.append(joint.joint_type)
                self.joint_lower_limits.append(joint.joint_limits.lower)
                self.joint_upper_limits.append(joint.joint_limits.upper)
                self.joint_vel_limits.append(joint.joint_limits.velocity)
                self.joint_acc_limits.append(joint.joint_limits.acceleration)
                self.joint_names.append(joint.joint_identifier.name)
                self.joint_uuids.append(joint.joint_identifier.uuid)
                
            # convert them to numpy arrays
            self.joint_types = np.asarray(self.joint_types)
            self.joint_lower_limits = np.asarray(self.joint_lower_limits)
            self.joint_upper_limits = np.asarray(self.joint_upper_limits)
            self.joint_vel_limits = np.asarray(self.joint_vel_limits)
            self.joint_acc_limits = np.asarray(self.joint_acc_limits)                

            self.num_joints = len(self.joint_info)
        else:
            # Give an error message to show that the robot is not connected
            print("Assign robot details failed. Robot is not connected to JogCartesianSpace service yet!")





def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-jogCartesianSpace-service", 8891) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginJogCartesianSpace")

        # create object
        JogCartesianSpace_inst = JogCartesianSpace_impl()
        # register service with service name "Vision", type "experimental.pluginVision.Vision", actual object: Vision_inst
        RRN.RegisterService("JogCartesianSpace","experimental.pluginJogCartesianSpace.JogCartesianSpace",JogCartesianSpace_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginJogCartesianSpace Server started, press enter to quit...")
        else:
            raw_input("pluginJogCartesianSpace Server started, press enter to quit...")

if __name__ == '__main__':
    main()