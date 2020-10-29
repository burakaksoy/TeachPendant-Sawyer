import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect to robot 
import numpy as np

import general_robotics_toolbox as rox
from qpsolvers import solve_qp

from vel_emulate_sub import EmulatedVelocityControl
import time

class JogCartesianSpace_impl(object):
    def __init__(self):
        self.url_robot = None
        self.robot_sub = None
        self.robot = None ## RR robot object
        self.robot_rox = None #Robotics Toolbox robot object

        # Incremental difference amounts to jog in cartesian space
        self.move_distance = 0.05 # meters
        self.rotate_angle = np.deg2rad(1) # radians

        self.pose_at_command = None 
        self.num_jog_command = 0

        self.dt = 0.02 #seconds, amount of time continuosly jog joints


    def reset(self):
        # Stop the joints first due to safety
        self.stop_joints()

        self.url_robot = None
        self.robot = None ## RR robot object
        self.robot_rox = None #Robotics Toolbox robot object

        self.pose_at_command = None 
        self.num_jog_command = 0

    def jog_cartesian2(self, P_axis, R_axis):
        print("Jog Joints2 is called")

        if self.pose_at_command is None:
            print("You need to call prepare_jog() function before calling jog_cartesian(P_axis,R_axis) function")
            return
        else:
            self.num_jog_command += 1
            print("num: " + str(self.num_jog_command))


        if self.robot is not None:
            print("Jog in Cartesian Space with command P_axis" + str(P_axis) + "and R_axis"+ str(R_axis)) 

            if self.is_enabled_velocity_mode == False:
                ## Put the robot to POSITION mode
                self.robot.command_mode = self.halt_mode
                # time.sleep(0.1)
                self.robot.command_mode = self.position_mode
                # time.sleep(0.1)

            if self.is_enabled_velocity_mode == False:
                #enable velocity mode
                self.vel_ctrl.enable_velocity_mode()
                self.is_enabled_velocity_mode = True

            ## Jog the robot in cartesian space
            # Update the end effector pose info
            # pose = self.get_current_pose()
            pose = self.pose_at_command 
            # Get the corresponding joint angles at that time
            # d_q = self.get_current_joint_positions()

            # Calculate desired pose
            Rd = pose.R
            pd = pose.p                
            # if P_axis is not None:
            zero_vec = np.array(([0.,0.,0.]))
            if not np.array_equal(P_axis, zero_vec):
                # Desired Position
                # pd = pd + Rd.dot( self.move_distance * P_axis)
                pd = pd + Rd.dot(self.num_jog_command * self.move_distance * P_axis)
            if not np.array_equal(R_axis, zero_vec):
                # R = rox.rot(np.array(([1.],[0.],[0.])), 0.261799)
                # R = rox.rot(R_axis, self.rotate_angle)
                R = rox.rot(R_axis, self.num_jog_command * self.rotate_angle)
                # Desired Orientation
                Rd = Rd.dot(R) # Rotate

            try:
                # calculate the required joint speeds (q_dot) to achive desired pose
                # qdot = self.update_qdot(Rd,pd,d_q)
                qdot = self.update_qdot(Rd,pd)

                now=time.time()
                while time.time()- now < self.dt:
                    self.vel_ctrl.set_velocity_command(qdot)
            except:
                print("Specified joints might be out of range")
                import traceback
                print(traceback.format_exc())
                # raise

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogCartesianSpace service yet!")

    def jog_cartesian_with_speed(self,P_axis,R_axis_angles, speed_perc):
        print("Jog Cartesian with speed percentage is called")

        if self.pose_at_command is None:
            print("You need to call prepare_jog() function before calling jog_cartesian(P_axis,R_axis) function")
            return
        else:
            self.num_jog_command += 1
            print("num: " + str(self.num_jog_command))

        if self.robot is not None:
            print("Jog in Cartesian Space with command P_axis" + str(P_axis) + "and R_axis_angles"+ str(R_axis_angles)) 

            ## Put the robot to jogging mode
            self.robot.command_mode = self.halt_mode
            # time.sleep(0.1)
            self.robot.command_mode = self.jog_mode
            # time.sleep(0.1)

            ## Jog the robot in cartesian space
            # Update the end effector pose info
            # pose = self.get_current_pose()
            pose = self.pose_at_command 
            
            Rd = pose.R
            pd = pose.p
                
            # if P_axis is not None:
            zero_vec = np.array(([0.,0.,0.]))
            # if not np.array_equal(P_axis, zero_vec):
            #     # Desired Position
            #     # pd = pd + Rd.dot(P_axis)
            pd = P_axis # Given P input is directly the desired end effector position
            
            # if not np.array_equal(R_axis_angles, zero_vec):
            # R = rox.rot(np.array(([1.],[0.],[0.])), 0.261799)
            # Calculate desired rotation from the euler angles (R_axis_angles)
            Rx = rox.rot(np.array(([1.],[0.],[0.])), R_axis_angles[0])
            Ry = rox.rot(np.array(([0.],[1.],[0.])), R_axis_angles[1])
            Rz = rox.rot(np.array(([0.],[0.],[1.])), R_axis_angles[2])

            R = Rz @ Ry @ Rx
            # Desired Orientation
            # Rd = Rd.dot(R) # Rotate
            Rd = R # calculated orientation is directly the desired orientation

            try:
                # Update desired inverse kineamtics info
                # joint_angles, converged = self.update_ik_info(Rd,pd)
                joint_angles, converged = self.update_ik_info2(Rd,pd)

                if not converged:
                    print("Inverse Kinematics Algo. Could not Converge")
                    # raise
                elif not (joint_angles < self.joint_upper_limits).all() or not (joint_angles > self.joint_lower_limits).all():
                    print("Specified joints are out of range")
                    # raise
                else:
                    wait = True
                    self.robot.jog_freespace(joint_angles, float(speed_perc)*0.01*self.joint_vel_limits, wait)
            except:
                print("Specified joints might be out of range")
                import traceback
                print(traceback.format_exc())
                # raise

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogCartesianSpace service yet!")


    def jog_cartesian_relative_with_speed(self,P_axis,R_axis_angles, speed_perc):
        print("Jog Cartesian relatively with speed percentage is called")

        if self.pose_at_command is None:
            print("You need to call prepare_jog() function before calling jog_cartesian(P_axis,R_axis) function")
            return
        else:
            self.num_jog_command += 1
            print("num: " + str(self.num_jog_command))

        if self.robot is not None:
            print("Jog in Cartesian Space with command P_axis" + str(P_axis) + "and R_axis_angles"+ str(R_axis_angles)) 

            ## Put the robot to jogging mode
            self.robot.command_mode = self.halt_mode
            # time.sleep(0.1)
            self.robot.command_mode = self.jog_mode
            # time.sleep(0.1)

            ## Jog the robot in cartesian space
            # Update the end effector pose info
            # pose = self.get_current_pose()
            pose = self.pose_at_command 
            
            Rd = pose.R
            pd = pose.p
                
            # if P_axis is not None:
            zero_vec = np.array(([0.,0.,0.]))
            if not np.array_equal(P_axis, zero_vec):
                # Desired Position
                pd = pd + Rd.dot(P_axis)
            if not np.array_equal(R_axis_angles, zero_vec):
                # R = rox.rot(np.array(([1.],[0.],[0.])), 0.261799)
                # Calculate desired rotation from the euler angles (R_axis_angles)
                Rx = rox.rot(np.array(([1.],[0.],[0.])), R_axis_angles[0])
                Ry = rox.rot(np.array(([0.],[1.],[0.])), R_axis_angles[1])
                Rz = rox.rot(np.array(([0.],[0.],[1.])), R_axis_angles[2])

                R = Rz @ Ry @ Rx
                # Desired Orientation
                Rd = Rd.dot(R) # Rotate

            try:
                # Update desired inverse kineamtics info
                # joint_angles, converged = self.update_ik_info(Rd,pd)
                joint_angles, converged = self.update_ik_info2(Rd,pd)

                if not converged:
                    print("Inverse Kinematics Algo. Could not Converge")
                    # raise
                elif not (joint_angles < self.joint_upper_limits).all() or not (joint_angles > self.joint_lower_limits).all():
                    print("Specified joints are out of range")
                    # raise
                else:
                    wait = True
                    self.robot.jog_freespace(joint_angles, float(speed_perc)*0.01*self.joint_vel_limits, wait)
            except:
                print("Specified joints might be out of range")
                import traceback
                print(traceback.format_exc())
                # raise

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogCartesianSpace service yet!")

    def stop_joints(self):
        print("stop_joints is called")
        if self.robot is not None:
            if self.is_enabled_velocity_mode == False:
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
            print("Robot is not connected to JogCartesianSpace service yet!")        

    def jog_cartesian(self, P_axis, R_axis):
        print("Jog cartesian is called")

        if self.pose_at_command is None:
            print("You need to call prepare_jog() function before calling jog_cartesian(P_axis,R_axis) function")
            return
        else:
            self.num_jog_command += 1
            print("num: " + str(self.num_jog_command))


        if self.robot is not None:
            print("Jog in Cartesian Space with command P_axis" + str(P_axis) + "and R_axis"+ str(R_axis)) 

            ## Put the robot to jogging mode
            self.robot.command_mode = self.halt_mode
            # time.sleep(0.1)
            self.robot.command_mode = self.jog_mode
            # time.sleep(0.1)

            ## Jog the robot in cartesian space
            # Update the end effector pose info
            # pose = self.get_current_pose()
            pose = self.pose_at_command 
            
            Rd = pose.R
            pd = pose.p
                
            # if P_axis is not None:
            zero_vec = np.array(([0.,0.,0.]))
            if not np.array_equal(P_axis, zero_vec):
                # Desired Position
                pd = pd + Rd.dot(self.num_jog_command * self.move_distance * P_axis)
            if not np.array_equal(R_axis, zero_vec):
                # R = rox.rot(np.array(([1.],[0.],[0.])), 0.261799)
                R = rox.rot(R_axis, self.num_jog_command * self.rotate_angle)
                # Desired Orientation
                Rd = Rd.dot(R) # Rotate

            try:
                # Update desired inverse kineamtics info
                # joint_angles, converged = self.update_ik_info(Rd,pd)
                joint_angles, converged = self.update_ik_info2(Rd,pd)

                if not converged:
                    print("Inverse Kinematics Algo. Could not Converge")
                    # raise
                elif not (joint_angles < self.joint_upper_limits).all() or not (joint_angles > self.joint_lower_limits).all():
                    print("Specified joints are out of range")
                    # raise
                else:
                    wait = True
                    relative = False
                    # self.robot.jog_joint(joint_angles, self.joint_vel_limits, relative, wait)
                    self.robot.jog_freespace(joint_angles, self.joint_vel_limits, wait)
            except:
                print("Specified joints might be out of range")
                import traceback
                print(traceback.format_exc())
                # raise

        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogCartesianSpace service yet!")

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
            print("Robot is connected to JogCartesianSpace service!")
        else:
            # Give an error that says the robot is already connected
            print("Robot is already connected to JogCartesianSpace service! Trying to connect again..")
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

            # Create roboics toolbox robot object as well
            self.create_robot_rox()

        else:
            # Give an error message to show that the robot is not connected
            print("Assign robot details failed. Robot is not connected to JogCartesianSpace service yet!")

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

        
        pose = rox.fwdkin(self.robot_rox, d_q) # Returns as pose.R and pose.p, loo rox for details
        return pose

    def update_ik_info(self, R_d, p_d): # inverse kinematics with least squares minimization
        # R_d, p_d: Desired orientation and position
        d_q = self.get_current_joint_positions()
        
        q_cur = d_q # initial guess on the current joint angles
        q_cur = q_cur.reshape((self.num_joints,1)) 
        
        epsilon = 0.001 # Damping Constant
        Kq = epsilon * np.eye(len(q_cur)) # small value to make sure positive definite used in Damped Least Square
        # print_div( "<br> Kq " + str(Kq) ) # DEBUG
        
        max_steps = 200 # number of steps to for convergence
        
        # print_div( "<br> q_cur " + str(q_cur) ) # DEBUG
        
        itr = 0 # Iterations
        converged = False
        while itr < max_steps and not converged:
        
            pose = rox.fwdkin(self.robot_rox,q_cur)
            R_cur = pose.R
            p_cur = pose.p
            
            #calculate current Jacobian
            J0T = rox.robotjacobian(self.robot_rox,q_cur)
            
            # Transform Jacobian to End effector frame from the base frame
            Tr = np.zeros((6,6))
            Tr[:3,:3] = R_cur.T 
            Tr[3:,3:] = R_cur.T
            J0T = Tr @ J0T
            # print_div( "<br> J0T " + str(J0T) ) # DEBUG
            
                          
            # Error in position and orientation
            # ER = np.matmul(R_cur, np.transpose(R_d))
            ER = np.matmul(np.transpose(R_d),R_cur)
            #print_div( "<br> ER " + str(ER) ) # DEBUG
            EP = R_cur.T @ (p_cur - p_d)                         
            #print_div( "<br> EP " + str(EP) ) # DEBUG
            
            #decompose ER to (k,theta) pair
            k, theta = rox.R2rot(ER)                  
            # print_div( "<br> k " + str(k) ) # DEBUG
            # print_div( "<br> theta " + str(theta) ) # DEBUG
            
            ## set up s for different norm for ER
            # s=2*np.dot(k,np.sin(theta)) #eR1
            # s = np.dot(k,np.sin(theta/2))         #eR2
            s = np.sin(theta/2) * np.array(k)         #eR2
            # s=2*theta*k              #eR3
            # s=np.dot(J_phi,phi)              #eR4
            # print_div( "<br> s " + str(s) ) # DEBUG         
                     
            alpha = 1 # Step size        
            # Damped Least square for iterative incerse kinematics   
            delta = alpha * (np.linalg.inv(Kq + J0T.T @ J0T ) @ J0T.T @ np.hstack((s,EP)).T )
            # print_div( "<br> delta " + str(delta) ) # DEBUG
            
            q_cur = q_cur - delta.reshape((self.num_joints,1))
            
            # Convergence Check
            converged = (np.hstack((s,EP)) < 0.0001).all()
            # print_div( "<br> converged? " + str(converged) ) # DEBUG
            
            itr += 1 # Increase the iteration
        
        # joints_text=""
        # for i in q_cur:
        #     joints_text+= "(%.3f, %.3f) " % (np.rad2deg(i), i)   
        # print_div_ik_info(str(rox.Transform(R_d,p_d)) +"<br>"+ joints_text +"<br>"+ str(converged) + ", itr = " + str(itr))
        return q_cur, converged

    def update_ik_info2(self, R_d, p_d): # inverse kinematics that uses QP solver
        # R_d, p_d: Desired orientation and position
        d_q = self.get_current_joint_positions()
        
        q_cur = d_q # initial guess on the current joint angles
        q_cur = q_cur.reshape((self.num_joints,1)) 
        
        epsilon = 0.01 # Damping Constant #0.001
        Kq = epsilon * np.eye(len(q_cur)) # small value to make sure positive definite used in Damped Least Square
        # print_div( "<br> Kq " + str(Kq) ) # DEBUG
        
        max_steps = 200 # number of steps to for convergence
        
        # print_div( "<br> q_cur " + str(q_cur) ) # DEBUG
        
        itr = 0 # Iterations
        converged = False
        while itr < max_steps and not converged:
            
            pose = rox.fwdkin(self.robot_rox,q_cur)
            R_cur = pose.R
            p_cur = pose.p
            
            #calculate current Jacobian
            J0T = rox.robotjacobian(self.robot_rox,q_cur)
            
            # Transform Jacobian to End effector frame from the base frame
            Tr = np.zeros((6,6))
            Tr[:3,:3] = R_cur.T 
            Tr[3:,3:] = R_cur.T
            J0T = Tr @ J0T
            
            Jp=J0T[3:,:]
            JR=J0T[:3,:]                      #decompose to position and orientation Jacobian
            
            # Error in position and orientation
            # ER = np.matmul(R_cur, np.transpose(R_d))
            ER = np.matmul(np.transpose(R_d),R_cur)
            #print_div( "<br> ER " + str(ER) ) # DEBUG

            # EP = p_cur - p_d                         
            EP = R_cur.T @ (p_cur - p_d)                         
            #print_div( "<br> EP " + str(EP) ) # DEBUG

            #decompose ER to (k,theta) pair
            k, theta = rox.R2rot(ER)                  
            # print_div( "<br> k " + str(k) ) # DEBUG
            # print_div( "<br> theta " + str(theta) ) # DEBUG
            
            ## set up s for different norm for ER
            # s=2*np.dot(k,np.sin(theta)) #eR1
            # s = np.dot(k,np.sin(theta/2))         #eR2
            s = np.sin(theta/2) * np.array(k)         #eR2
            # s=2*theta*k              #eR3
            # s=np.dot(J_phi,phi)              #eR4
            # print_div( "<br> s " + str(s) ) # DEBUG         

            Kp = np.eye(3)
            KR = np.eye(3)        #gains for position and orientation error
            
            vd = - Kp @ EP
            wd = - KR @ s
            
            w=10000             #set the weight between orientation and position

            H = Jp.T @ Jp + w*JR.T @ JR + Kq 
            H = (H + H.T)/2

            f= -(Jp.T @ vd + w* JR.T @ wd)               #setup quadprog parameters
            #     quadratic function with +/-qddot (1 rad/s^2) as upper/lower bound 

            qdot_star = solve_qp(H, f) 

            # find best step size to take
            # alpha=fminbound(min_alpha,0,1,args=(q_cur,qdot_star,Sawyer_def,Rd,pd,w,Kp))
            alpha = 0.8 # Step size    # 1.0    
            delta = alpha * qdot_star 
            # print_div( "<br> delta " + str(delta) ) # DEBUG
            
            q_cur = q_cur + delta.reshape((self.num_joints,1))
            
            # Convergence Check
            converged = (np.hstack((s,EP)) < 0.0001).all()
            # print_div( "<br> converged? " + str(converged) ) # DEBUG
            # print( "converged? " + str(converged) ) # DEBUG
            
            itr += 1 # Increase the iteration
        
        # joints_text=""
        # for i in q_cur:
        #     joints_text+= "(%.3f, %.3f) " % (np.rad2deg(i), i)   
        # print_div_ik_info(str(rox.Transform(R_d,p_d)) +"<br>"+ joints_text +"<br>"+ str(converged) + ", itr = " + str(itr))
        return q_cur, converged

    # def update_qdot(self, R_d, p_d,d_q): # inverse kinematics that uses QP solver
    def update_qdot(self, R_d, p_d): # inverse kinematics that uses QP solver
        # R_d, p_d: Desired orientation and position
        # d_q current joint angles
        d_q = self.get_current_joint_positions()
        
        q_cur = d_q # initial guess on the current joint angles
        q_cur = d_q.reshape((self.num_joints,1)) 
        
        epsilon = 0.01 # Damping Constant #0.001
        Kq = epsilon * np.eye(len(q_cur)) # small value to make sure positive definite used in Damped Least Square
        # print_div( "<br> Kq " + str(Kq) ) # DEBUG
        
        
        # print_div( "<br> q_cur " + str(q_cur) ) # DEBUG
            
        pose = rox.fwdkin(self.robot_rox,q_cur)
        R_cur = pose.R
        p_cur = pose.p
        
        #calculate current Jacobian
        J0T = rox.robotjacobian(self.robot_rox,q_cur)
        
        # Transform Jacobian to End effector frame from the base frame
        Tr = np.zeros((6,6))
        Tr[:3,:3] = R_cur.T 
        Tr[3:,3:] = R_cur.T
        J0T = Tr @ J0T
        
        Jp=J0T[3:,:]
        JR=J0T[:3,:]                      #decompose to position and orientation Jacobian
        
        # Error in position and orientation
        # ER = np.matmul(R_cur, np.transpose(R_d))
        ER = np.matmul(np.transpose(R_d),R_cur)
        #print_div( "<br> ER " + str(ER) ) # DEBUG

        # EP = p_cur - p_d                         
        EP = R_cur.T @ (p_cur - p_d)                         
        #print_div( "<br> EP " + str(EP) ) # DEBUG

        #decompose ER to (k,theta) pair
        k, theta = rox.R2rot(ER)                  
        # print_div( "<br> k " + str(k) ) # DEBUG
        # print_div( "<br> theta " + str(theta) ) # DEBUG
        
        ## set up s for different norm for ER
        # s=2*np.dot(k,np.sin(theta)) #eR1
        # s = np.dot(k,np.sin(theta/2))         #eR2
        s = np.sin(theta/2) * np.array(k)         #eR2
        # s=2*theta*k              #eR3
        # s=np.dot(J_phi,phi)              #eR4
        # print_div( "<br> s " + str(s) ) # DEBUG         

        Kp = np.eye(3)
        KR = np.eye(3)        #gains for position and orientation error
        
        vd = - Kp @ EP
        wd = - KR @ s
        
        w= 10000            #set the weight between orientation and position

        H = Jp.T @ Jp + w*JR.T @ JR + Kq 
        H = (H + H.T)/2

        f= -(Jp.T @ vd + w* JR.T @ wd)               #setup quadprog parameters
        #     quadratic function with +/-qddot (1 rad/s^2) as upper/lower bound 

        qdot_star = solve_qp(H, f) 

        q_dot = self.normalize_dq(qdot_star)
        
        return q_dot

    def normalize_dq(self,q):
        # q[:-1]=0.5*q[:-1]/(np.linalg.norm(q[:-1])) 
        q=q/(np.linalg.norm(q)) 
        return q 


    def prepare_jog(self):
        if self.robot is not None:
            self.pose_at_command = self.get_current_pose()
            self.num_jog_command = 0
        else:
            # Give an error message to show that the robot is not connected
            print("Robot is not connected to JogCartesianSpace service yet!")


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