import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect to robot 
import numpy as np


from vel_emulate_sub import EmulatedVelocityControl
import time

class JogJointSpace_impl(object):
    def __init__(self):
        self.url_robot = None
        self.robot_sub = None
        self.robot = None
        self.robot_rox = None #Robotics Toolbox robot object

        self.degree_diff = 10 # in degrees
        self.dt = 0.01 #seconds, amount of time continuosly jog joints


    def reset(self):
        # Stop the joints first due to safety
        self.stop_joints()

        self.url_robot = None
        self.robot_sub = None
        self.robot = None ## RR robot object
        self.robot_rox = None #Robotics Toolbox robot object



    def jog_joints2(self, q_i, sign):
        print("Jog Joints2 is called")
        if self.robot is not None:
            if self.robot.command_mode != self.position_mode:
                # Put the robot to POSITION mode
                self.robot.command_mode = self.halt_mode
                # time.sleep(0.1)
                self.robot.command_mode = self.position_mode
                # time.sleep(0.1)
            if self.is_enabled_velocity_mode == False:
                #enable velocity mode
                self.vel_ctrl.enable_velocity_mode()
                self.is_enabled_velocity_mode = True

            # Jog the robot
            if (self.num_joints < q_i):
                print("Currently Controlled Robot only have " + str(self.num_joints) + " joints..")
            else:
                joint_diff = np.zeros((self.num_joints,))
                joint_diff[q_i-1] = sign*np.deg2rad(self.degree_diff)

                # qdot=(joint_diff)/self.dt
                qdot=(joint_diff)/1.0 # Make joint speeds such that to take joint angle differences in 1 second

                now=time.time()
                while time.time()- now < self.dt:
                    self.vel_ctrl.set_velocity_command(2*qdot)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def stop_joints(self):
        print("stop_joints is called")
        if self.robot is not None:
            if self.robot.command_mode != self.position_mode:
                # Put the robot to POSITION mode
                self.robot.command_mode = self.halt_mode
                # time.sleep(0.1)
                self.robot.command_mode = self.position_mode
                # time.sleep(0.1)
            if self.is_enabled_velocity_mode == False:
                #enable velocity mode
                self.vel_ctrl.enable_velocity_mode()
                self.is_enabled_velocity_mode = True

            # stop the robot
            self.vel_ctrl.set_velocity_command(np.zeros((self.num_joints,)))

            # disable velocity mode
            self.vel_ctrl.disable_velocity_mode() 
            self.is_enabled_velocity_mode = False
        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def stop_joints2(self):
        print("stop_joints2 is called")
        if self.robot is not None:
            if self.robot.command_mode != self.jog_mode:
                # Put the robot to JOG mode
                self.robot.command_mode = self.halt_mode
                # time.sleep(0.1)
                self.robot.command_mode = self.jog_mode
                # time.sleep(0.1)

            # stop the robot
            self.robot.jog_joint(np.zeros((self.num_joints,)), 50*self.dt, True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")


    def jog_joints3(self, q_i, sign):
        print("Jog Joints3 is called")
        if self.robot is not None:
            try:
                if self.robot.command_mode != self.jog_mode:
                    # Put the robot to jogging mode
                    self.robot.command_mode = self.halt_mode
                    print(self.robot.command_mode)
                    # time.sleep(0.1)
                    self.robot.command_mode = self.jog_mode
                    # time.sleep(0.1)
                    print(self.robot.command_mode)

                # Jog the robot
                # # get the current joint angles
                cur_q = self.get_current_joint_positions()

                if (self.num_joints < q_i):
                    print("Currently Controlled Robot only have " + str(self.num_joints) + " joints..")
                else:
                    joint_vel = np.zeros((self.num_joints,))
                    joint_vel[q_i-1] = sign*self.joint_vel_limits[q_i-1]/10.0

                    self.jog_joints_with_limits2(cur_q, joint_vel,25*self.dt, False)
            except:
                # print("Specified joints might be out of range222")
                import traceback
                print(traceback.format_exc())
        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")


    def jog_joints(self, q_i, sign):
        print("Jog Joints is called")
        if self.robot is not None:
            if self.robot.command_mode != self.jog_mode:
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

                # self.jog_joints_with_limits((cur_q + joint_diff),(cur_q + joint_diff),joint_diff, self.joint_vel_limits,True,True)
                self.jog_joints_with_limits((cur_q + joint_diff), self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joints_with_limits(self,joint_position, max_velocity,wait=True):
        if not (joint_position < self.joint_upper_limits).all() or not (joint_position > self.joint_lower_limits).all():
            print("Specified joints might be out of range")
        else:
            try:
                if self.robot.command_mode != self.jog_mode:
                    # Put the robot to jogging mode
                    self.robot.command_mode = self.halt_mode
                    # time.sleep(0.1)
                    self.robot.command_mode = self.jog_mode
                    # time.sleep(0.1)

                # Trim joint positions according to number of joints
                joint_position = joint_position[:self.num_joints]
                # self.robot.jog_joint(joint_position, max_velocity, relative, wait)
                self.robot.jog_freespace(joint_position, max_velocity, wait)
            except:
                # print("Specified joints might be out of range222")
                import traceback
                print(traceback.format_exc())

    def jog_joints_with_limits2(self,joint_position, joint_velocity, timeout, wait=True):
        if not (joint_position <= self.joint_upper_limits).all() or not (joint_position >= self.joint_lower_limits).all():
            print("Specified joints might be out of range")
        else:
            try:
                if self.robot.command_mode != self.jog_mode:
                    # Put the robot to jogging mode
                    self.robot.command_mode = self.halt_mode
                    # time.sleep(0.1)
                    self.robot.command_mode = self.jog_mode
                    # time.sleep(0.1)

                # Trim joint positions according to number of joints
                joint_velocity = joint_velocity[:self.num_joints]
                # self.robot.jog_joint(joint_position, max_velocity, relative, wait)
                self.robot.jog_joint(joint_velocity, timeout, wait)
            except:
                # print("Specified joints might be out of range222")
                import traceback
                print(traceback.format_exc())

    def jog_joints_gamepad(self,joint_speed_constants):
        print("Jog Joints Gamepad is called")
        if self.robot is not None:
            if self.robot.command_mode != self.jog_mode:
                # Put the robot to jogging mode
                self.robot.command_mode = self.halt_mode
                # time.sleep(0.1)
                self.robot.command_mode = self.jog_mode
                # time.sleep(0.1)
            
            # get the current joint angles
            cur_q = self.get_current_joint_positions()

            # Trim joint speed constants accordingto number of joints
            joint_speed_constants = joint_speed_constants[:self.num_joints]

            signs = np.divide(np.abs(joint_speed_constants),joint_speed_constants)
            np.nan_to_num(signs, copy=False)

            joint_diff = np.ones((self.num_joints,))
            joint_diff = np.multiply(signs,np.deg2rad(self.degree_diff))

            # self.jog_joints_with_limits((cur_q + joint_diff),(cur_q + joint_diff),joint_diff, self.joint_vel_limits,True,True)
            self.jog_joints_with_limits((cur_q + joint_diff), self.joint_vel_limits,False)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joints_zeros(self):
        print("Jog Joints Zeros is called")
        if self.robot is not None:
            if self.robot.command_mode != self.jog_mode:
                # Put the robot to jogging mode
                self.robot.command_mode = self.halt_mode
                # time.sleep(0.1)
                self.robot.command_mode = self.jog_mode
                # time.sleep(0.1)
            
            self.jog_joints_with_limits(np.zeros((self.num_joints,)), self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joints_to_angles(self, joint_position):
        print("Jog Joints to Angles is called")
        # Similar to jog_joints_with_limits. But,
        # Moves the robot to the specified joint angles with max speed
        if self.robot is not None:
            if self.robot.command_mode != self.jog_mode:
                # Put the robot to jogging mode
                self.robot.command_mode = self.halt_mode
                # time.sleep(0.1)
                self.robot.command_mode = self.jog_mode
                # time.sleep(0.1)

            # print(joint_position[:self.num_joints])
            # print(joint_position)

            self.jog_joints_with_limits(joint_position[:self.num_joints], self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    # For blockly
    def jog_joints_to_angles_relative(self,diff_joint_position, speed_perc):
        print("Jog Joints to Angles relatively is called")
        if self.robot is not None:
            if self.robot.command_mode != self.jog_mode:
                # Put the robot to jogging mode
                self.robot.command_mode = self.halt_mode
                # time.sleep(0.1)
                self.robot.command_mode = self.jog_mode
                # time.sleep(0.1)

            # print(joint_position[:self.num_joints])
            # print(joint_position)

            # # get the current joint angles
            cur_q = self.get_current_joint_positions()
            diff_joint_position = diff_joint_position[:self.num_joints]

            self.jog_joints_with_limits((diff_joint_position+cur_q), float(speed_perc)*0.01*self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joint_to_angle(self, joint, position, speed_perc):
        print("Jog Joint to Angle is called")
        if self.robot is not None:
            if self.robot.command_mode != self.jog_mode:
                # Put the robot to jogging mode
                self.robot.command_mode = self.halt_mode
                # time.sleep(0.1)
                self.robot.command_mode = self.jog_mode
                # time.sleep(0.1)


            # # get the current joint angles
            cur_q = self.get_current_joint_positions()
            cur_q[joint] = position

            self.jog_joints_with_limits(cur_q, float(speed_perc)*0.01*self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")

    def jog_joints_to_angles2(self, joint_position, speed_perc):
        print("Jog Joints to Angles2 (2 = with speed) is called")
        # Similar to jog_joints_with_limits. But,
        # Moves the robot to the specified joint angles with max speed percentage
        if self.robot is not None:
            if self.robot.command_mode != self.jog_mode:
                # Put the robot to jogging mode
                self.robot.command_mode = self.halt_mode
                # time.sleep(0.1)
                self.robot.command_mode = self.jog_mode
                # time.sleep(0.1)

            self.jog_joints_with_limits(joint_position[:self.num_joints], float(speed_perc)*0.01*self.joint_vel_limits,True)

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogJointSpace service yet!")


    def connect2robot(self, url_robot):
        if self.robot is None:
            self.url_robot = url_robot

            # self.robot = RRN.ConnectService(self.url_robot) # connect to robot with the given url
            self.robot_sub = RRN.SubscribeService(self.url_robot)
            self.robot = self.robot_sub.GetDefaultClientWait(1)            
            
            # self.robot.reset_errors()
            # self.robot.enable()

            # Define Robot modes
            self.robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", self.robot)
            self.halt_mode = self.robot_const["RobotCommandMode"]["halt"]
            self.jog_mode = self.robot_const["RobotCommandMode"]["jog"]
            
            self.position_mode = self.robot_const["RobotCommandMode"]["position_command"]
            # self.trajectory_mode = self.robot_const["RobotCommandMode"]["trajectory"]

            self.assign_robot_details()

            # ---------------------------
            # self.robot_sub=RRN.SubscribeService(self.url_robot)
            # self.state_w = self.robot_sub.SubscribeWire("robot_state")    
            self.is_enabled_velocity_mode = False
            # self.cmd_w = self.robot_sub.SubscribeWire("position_command")
            # self.vel_ctrl = EmulatedVelocityControl(self.robot,self.state_w, self.cmd_w, self.dt)

            self.vel_ctrl = EmulatedVelocityControl(self.robot, self.dt)
            # ---------------------------
            
            # log that the robot is successfully connected  
            print("Robot is connected to JogJointSpace service!")
        else:
            # Give an error that says the robot is already connected
            print("Robot is already connected to JogJointSpace service! Trying to connect again..")
            self.reset()
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
    with RR.ServerNodeSetup("experimental.plugin-jogJointSpace-service", 8890) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginJogJointSpace")

        # create object
        JogJointSpace_inst = JogJointSpace_impl()
        # register service with service name "JogJointSpace", type "experimental.pluginJogJointSpace.JogJointSpace", actual object: JogJointSpace_inst
        RRN.RegisterService("JogJointSpace","experimental.pluginJogJointSpace.JogJointSpace",JogJointSpace_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginJogJointSpace Server started, press enter to quit...")
        else:
            raw_input("pluginJogJointSpace Server started, press enter to quit...")

if __name__ == '__main__':
    main()