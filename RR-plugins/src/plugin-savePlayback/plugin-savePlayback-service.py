import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect to robot 
import numpy as np

import general_robotics_toolbox as rox

class SavePlayback_impl(object):
    def __init__(self):
        self.url_robot = None
        self.robot = None ## RR robot object
        self.robot_rox = None #Robotics Toolbox robot object

        self.saved_joint_angles_lst = [] # List to store saved joint angle ndarrays
        self.saved_endeff_poses_lst = [] # List to store corresponding end eff. pose rox pose objects


    def save_cur_pose(self):
        print("save_cur_pose is called")
        if self.robot is not None:
            # get the current joint angles
            cur_q = self.get_current_joint_positions()
            # save it to this service's joint angles property list
            self.saved_joint_angles_lst.append(cur_q)
            # Calculate the corresponding end effector to that joint angles
            cur_pose = self.get_current_pose() # Returns as pose.R and pose.p, look rox for details
            # save it to this service's enfedd property list
            self.saved_endeff_poses_lst.append(cur_pose)

            print("save_cur_pose is SUCCESSFUL!")

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to SavePlayback service yet!")

    def go_sel_pose(self, index):
        print("go_sel_pose is called")
        if self.robot is not None:
            # get the desired pose from the list
            q_desired = self.saved_joint_angles_lst[index]
            print("Desired angles:" + str(q_desired))

            # Put the robot to jogging mode
            self.robot.command_mode = self.halt_mode
            self.robot.command_mode = self.jog_mode

            # call jogging function to go to desired joint angles
            self.robot.jog_joint(q_desired, self.joint_vel_limits, False, True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to SavePlayback service yet!")

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
            
            self.position_mode = self.robot_const["RobotCommandMode"]["velocity_command"]
            self.trajectory_mode = self.robot_const["RobotCommandMode"]["trajectory"]

            self.assign_robot_details()
            
            # log that the robot is successfully connected  
            print("Robot is connected to SavePlayback service!")
        else:
            # Give an error that says the robot is already connected
            print("Robot is already connected to SavePlayback service! Trying to connect again..")
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

            # Create roboics toolbox robot object as well
            self.create_robot_rox()


        else:
            # Give an error message to show that the robot is not connected
            print("Assign robot details failed. Robot is not connected to JogJointSpace service yet!")

    def create_robot_rox(self):
        chains = self.robot_info.chains # Get RobotKinChainInfo
        print("1.1")
        self.H = chains[0].H # Axes of the joints, 3xN 
        print("1.2")
        self.P = chains[0].P # P vectors between joint centers (Product of Exponenetials Convention)
        print("1.3")
        self.H_shaped = np.zeros((3, self.num_joints))
        itr = 0
        for i in self.H:
            self.H_shaped[:,itr] = (i[0],i[1],i[2])
            itr += 1

        self.P_shaped = np.zeros((3, self.num_joints+1))  
        itr = 0  
        for i in self.P:
            self.P_shaped[:,itr] = (i[0],i[1],i[2])
            itr += 1

        # create the robot from toolbox
        # robot = rox.Robot(H,P,joint_types-1,joint_lower_limits,joint_upper_limits,joint_vel_limits,joint_acc_limits)
        self.robot_rox = rox.Robot(self.H_shaped,self.P_shaped,self.joint_types-1)

    def get_current_joint_positions(self):
        cur_robot_state = self.robot.robot_state.PeekInValue()    
        cur_q = cur_robot_state[0].joint_position
        return cur_q # in radian ndarray

    def get_current_pose(self):
        d_q = self.get_current_joint_positions()
        pose = rox.fwdkin(self.robot_rox, d_q) # Returns as pose.R and pose.p, look rox for details
        return pose

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-savePlayback-service", 8894) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginSavePlayback")

        # create object
        SavePlayback_inst = SavePlayback_impl()
        # register service with service name "SavePlayback", type "experimental.pluginSavePlayback.SavePlayback", actual object: SavePlayback_inst
        RRN.RegisterService("SavePlayback","experimental.pluginSavePlayback.SavePlayback",SavePlayback_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginSavePlayback Server started, press enter to quit...")
        else:
            raw_input("pluginSavePlayback Server started, press enter to quit...")

if __name__ == '__main__':
    main()