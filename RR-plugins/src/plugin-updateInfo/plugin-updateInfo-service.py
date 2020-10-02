import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect to robot 
import numpy as np

import general_robotics_toolbox as rox
import math

class UpdateInfo_impl(object):
    def __init__(self):
        self.url_robot = None
        self.robot = None ## RR robot object
        self.robot_rox = None #Robotics Toolbox robot object

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
        
        self.H = chains[0].H # Axes of the joints, 3xN     
        self.P = chains[0].P # P vectors between joint centers (Product of Exponenetials Convention)
    
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
    def current_pose_str(self):
        # Returns the pose as a string="[x, y, z];[psi, theta, phi]"
        # where x y z position vector wrt base in meters
        # psi theta phi x y z Euler angles of end effector wrt base in radians for Rz * Ry * Rx
        cur_pose = self.get_current_pose()

        # Calculate euler ZYX angles from pose and write them into:
        x,y,z = self.euler_angles_from_rotation_matrix(cur_pose.R)
        str_rx = "%.2f" % (np.rad2deg(x))
        str_ry = "%.2f" % (np.rad2deg(y))
        str_rz = "%.2f" % (np.rad2deg(z))
        str_px = "%.3f" % (cur_pose.p[0])
        str_py = "%.3f" % (cur_pose.p[1])
        str_pz = "%.3f" % (cur_pose.p[2])
        xyz_positi_str = "[" + str_px + ", " + str_py + ", " + str_pz + "];"
        xyz_orient_str = "[" + str_rx + ", " + str_ry + ", " + str_rz + "]"

        pose_str = xyz_positi_str + xyz_orient_str
        return pose_str

    def state_flags_str(self):
        cur_robot_state = self.robot.robot_state.PeekInValue()
        state_flags_enum = self.robot_const['RobotStateFlags']

        flags_text =""
        for flag_name, flag_code in state_flags_enum.items():
            if flag_code & cur_robot_state[0].robot_state_flags != 0:
                flags_text += flag_name + " "

        return flags_text

    def current_joint_angles_str(self):
        cur_q = self.get_current_joint_positions()

        joints_text=""
        for i in cur_q:
            # joints_text+= "%.2f, %.2f<br>" % (np.rad2deg(i), i)
            joints_text+= "%.2f," % (np.rad2deg(i))
        # joints_text += ", shape = " + str(cur_q.shape) + "<br>" # DEBUG
        
        # print_div_j_info(type(cur_q).__name__)
        return joints_text

    def joint_limits_str_array(self):
        joint_lower_limits_text = ""    
        for i in self.joint_lower_limits:
            joint_lower_limits_text += "%.2f " % (np.rad2deg(i))
            
        joint_upper_limits_text = ""
        for i in self.joint_upper_limits:
            joint_upper_limits_text += "%.2f " % (np.rad2deg(i))

        return [joint_lower_limits_text,joint_upper_limits_text]

    def joint_num_type_vel_acc_name_str(self):
        return str(self.num_joints) +"<br>"+ str(self.joint_types) +"<br>"+ str(self.joint_vel_limits) +"<br>"+ str(self.joint_acc_limits) +"<br>"+ str(self.joint_names) # +"<br>"+ str(self.joint_uuids))

    def kinematics_str(self):
        H_text = "H (axis):<br>"
        itr = 0
        for i in self.H:
            H_text+= "|" + " %.1f " %i[0] + " %.1f " %i[1] + " %.1f " %i[2]  + "|<br> "
            itr += 1

        P_text = "P (in meters):<br>"
        itr = 0  
        for i in self.P:
            P_text+= "|" + " %.3f " %i[0] + " %.3f " %i[1] + " %.3f " %i[2]  + " |<br> "
            itr += 1

        HnP_text = H_text + "<br>" + P_text
        return HnP_text





def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-updateInfo-service", 8895) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginUpdateInfo")

        # create object
        UpdateInfo_inst = UpdateInfo_impl()
        # register service with service name "UpdateInfo", type "experimental.pluginUpdateInfo.UpdateInfo", actual object: UpdateInfo_inst
        RRN.RegisterService("UpdateInfo","experimental.pluginUpdateInfo.UpdateInfo",UpdateInfo_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginUpdateInfo Server started, press enter to quit...")
        else:
            raw_input("pluginUpdateInfo Server started, press enter to quit...")

if __name__ == '__main__':
    main()