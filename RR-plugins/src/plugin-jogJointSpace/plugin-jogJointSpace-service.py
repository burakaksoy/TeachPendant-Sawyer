import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect to robot 
import numpy as np

class JogJointSpace_impl(object):
    def __init__(self):
        self.url_robot = None
        self.robot = None

        self.degree_diff = 1

    def jog_joints(self, q_i, sign):
        print("Jog Joints is called")
        if self.robot is not None:
            # Put the robot to jogging mode
            self.robot.command_mode = self.halt_mode
            # time.sleep(0.1)
            self.robot.command_mode = self.jog_mode
            # time.sleep(0.1)
            # Jog the robot
            # # get the current joint angles
            cur_q = self.get_current_joint_positions()

            if (self.num_joints < q_i):
                print("Currently Controlled Robot only have " + str(self.num_joints) + " joints..")
            else:
                joint_diff = np.zeros((self.num_joints,))
                joint_diff[q_i-1] = sign*np.deg2rad(self.degree_diff)

                self.jog_joints_with_limits((cur_q + joint_diff),(cur_q + joint_diff),joint_diff, self.joint_vel_limits,True,True)
                
                # if not ((cur_q + joint_diff) < self.joint_upper_limits).all() or not ((cur_q + joint_diff) > self.joint_lower_limits).all():
                #     print("Specified joints might be out of range")
                # else:
                #     try:
                #         self.robot.jog_joint(joint_diff, self.joint_vel_limits, True, True)
                #     except:
                #         print("Specified joints might be out of range222")

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joints_with_limits(self,up_limits,low_limits, joint_position, max_velocity,relative=False,wait=True):
        if not (up_limits < self.joint_upper_limits).all() or not (low_limits > self.joint_lower_limits).all():
            print("Specified joints might be out of range")
        else:
            try:
                self.robot.jog_joint(joint_position, max_velocity, relative, wait)
            except:
                print("Specified joints might be out of range222")

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
            print("Robot is connected to JogJointSpace service!")
        else:
            # Give an error that says the robot is already connected
            print("Robot is already connected to JogJointSpace service! Trying to connect again..")
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
            print("Assign robot details failed. Robot is not connected to JogJointSpace service yet!")

    def get_current_joint_positions(self):
        cur_robot_state = self.robot.robot_state.PeekInValue()    
        cur_q = cur_robot_state[0].joint_position
        return cur_q # in radian ndarray

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-jogJointSpace-service", 8888) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginJogJointSpace")

        # create object
        JogJointSpace_inst = JogJointSpace_impl()
        # register service with service name "JogJointSpace", type "experimental.pluginJogJointSpace.JogJointSpace", actual object: JogJointSpace_inst
        RRN.RegisterService("JogJointSpace","experimental.pluginJogJointSpace.JogJointSpace",JogJointSpace_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("Server started, press enter to quit...")
        else:
            raw_input("Server started, press enter to quit...")

if __name__ == '__main__':
    main()