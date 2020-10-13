import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect to robot 
import numpy as np
import time

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
            # self.robot.jog_joint(q_desired, self.joint_vel_limits, False, True)
            self.robot.jog_freespace(q_desired, self.joint_vel_limits, True)

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

    def playback_poses(self, num_loops, joint_vel_ratio, t_complete):
        print("Playback poses function is called.")
        # Import Necessary Structures
        JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",self.robot)
        JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",self.robot)
        # We have joint_names as self.joint_names
        # we have poses list as self.saved_joint_angles_lst or self.saved_endeff_poses_lst

        # Create waypoints array 
        waypoints = []

        index = 0 # index in saved poses list
        while index < (len(self.saved_joint_angles_lst)):
            joint_angles = self.saved_joint_angles_lst[index]
            if not (joint_angles < self.joint_upper_limits).all() or not (joint_angles > self.joint_lower_limits).all():
                print("Specified joints are out of range.")
                return
            else:
                # Go to initial waypoint
                if index == 0:
                    self.go_sel_pose(index)
                    # RRN.Sleep(2,None)
                    time.sleep(2)
                
                # Define the time to be at that waypoint
                t = float(index)*(float(t_complete)/float(len(self.saved_joint_angles_lst)))
                # Add the joint angles to waypoints array
                wp = JointTrajectoryWaypoint()
                wp.joint_position = joint_angles # (j_end - j_start)*(float(i)/10.0) + j_start
                wp.time_from_start = t
                waypoints.append(wp)
            index += 1

        # Complete the loop, Add the 1st joint angles to waypoints array again
        t = float(index)*(float(t_complete)/float(len(self.saved_joint_angles_lst)))
        # Add the joint angles to waypoints array 
        wp = JointTrajectoryWaypoint()
        wp.joint_position = waypoints[0].joint_position
        wp.time_from_start = t
        waypoints.append(wp)

        # Put robot to trajectory mode
        self.robot.command_mode = self.halt_mode
        self.robot.command_mode = self.trajectory_mode

        # Create the trajectory
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.waypoints = waypoints
        self.robot.speed_ratio = joint_vel_ratio
        try:
            # Execute the trajectory number of loops times
            i = 0 # loop
            while i < num_loops:
                traj_gen = self.robot.execute_trajectory(traj)
                while (True):
                    t = time.time()
                    try:
                        res = traj_gen.Next()        
                    except RR.StopIterationException:
                        break
                i += 1
                print("Loop:" + str(i) + "is done..")
        except:
            import traceback
            print(traceback.format_exc())
            print("Robot accelaration or velocity limits might be out of range. Increase the loop time or slow down the speed ratio<br>")

        # # ##############################################################
        # async def async_playback_poses_func():
        #     global d, num_joints, joint_lower_limits, joint_upper_limits, joint_vel_limits
            
        #     # Import Necessary Structures
        #     # global JointTrajectoryWaypoint, JointTrajectory
        #     JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",d)
        #     JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",d)

        #     # Get Joint Names
        #     robot_info = await d.async_get_robot_info(None)
        #     joint_names = [j.joint_identifier.name for j in robot_info.joint_info]

        #     # Get elements, poses and paramaters from web interface
        #     poses_list = document.getElementById("saved_poses_list")
        #     num_loops_elem = document.getElementById("num_loops_in")
        #     num_loops = int(num_loops_elem.value)
        #     joint_vel_range = document.getElementById("joint_vel_range")
        #     joint_vel_ratio = float(joint_vel_range.value)/100.0

        #     time_loops_elem = document.getElementById("time_loops_in")
        #     time_loops = float(time_loops_elem.value)

        #     # Time to complete the playback
        #     t_complete = time_loops #seconds
            
        #     # Create waypoints array 
        #     waypoints = []
        #     if poses_list.length >= 4:
        #         index = 0 # index in saved poses list
        #         while index < (poses_list.length):
        #             sel_pose = poses_list.options[index].value # angles as str
        #             joint_angles = np.fromstring(sel_pose, dtype=float, sep=',')*np.deg2rad(1) # in rad

        #             if not (joint_angles < joint_upper_limits).all() or not (joint_angles > joint_lower_limits).all():
        #                 print_div("Specified joints are out of range<br>")
        #                 return
        #             else:
        #                 # Go to initial waypoint
        #                 if index == 0:
        #                     await d.async_jog_joint(joint_angles, joint_vel_limits, False, True,None)
        #                     await RRN.AsyncSleep(2,None)

        #                 # Define the time to be at that waypoint
        #                 t = float(index)*(float(t_complete)/float(poses_list.length))
        #                 # Add the joint angles to waypoints array
        #                 wp = JointTrajectoryWaypoint()
        #                 wp.joint_position = joint_angles # (j_end - j_start)*(float(i)/10.0) + j_start
        #                 wp.time_from_start = t
        #                 waypoints.append(wp)

        #             index += 1
        #         # Complete the loop, Add the 1st joint angles to waypoints array again
        #         t = float(index)*(float(t_complete)/float(poses_list.length))
        #         # Add the joint angles to waypoints array
        #         wp = JointTrajectoryWaypoint()
        #         wp.joint_position = waypoints[0].joint_position
        #         wp.time_from_start = t
        #         waypoints.append(wp)

        #     else:
        #         print_div("You need at least 4 different points. Add some poses to Saved Poses and try again<br>")
        #         # # Put robot to jogging mode back
        #         # await d.async_set_command_mode(halt_mode,None,5)
        #         # await RRN.AsyncSleep(0.01,None)
        #         # await d.async_set_command_mode(jog_mode,None,5)
        #         # await RRN.AsyncSleep(0.01,None)
        #         return

        #     # Put robot to trajectory mode
        #     await d.async_set_command_mode(halt_mode,None,5)
        #     await RRN.AsyncSleep(0.01,None)
        #     await d.async_set_command_mode(trajectory_mode,None,5)
        #     await RRN.AsyncSleep(0.01,None)

        #     # Create the trajectory
        #     traj = JointTrajectory()
        #     traj.joint_names = joint_names
        #     traj.waypoints = waypoints
        #     d.async_set_speed_ratio(joint_vel_ratio,None,5)

        #     try:
        #         # Execute the trajectory number of loops times
        #         i = 0 # loop
        #         while i < num_loops:
        #             traj_gen = await d.async_execute_trajectory(traj, None)
                
        #             while (True):
        #                 t = time.time()

        #                 # robot_state = state_w.InValue
        #                 try:
        #                     # print_div("Here")
        #                     res = await traj_gen.AsyncNext(None, None)        
        #                     # print_div(res.action_status)
        #                 except RR.StopIterationException:
        #                     break

        #             i += 1
        #             print_div("Loop:" + str(i) + "is done..<br>")
        #     except:
        #         # import traceback
        #         # print_div(traceback.format_exc())
        #         print_div("Robot accelaration or velocity limits might be out of range. Increase the loop time or slow down the speed ratio<br>")
        #         # return
        #         # raise

        #     # Put robot to jogging mode back
        #     await d.async_set_command_mode(halt_mode,None,5)
        #     await RRN.AsyncSleep(0.01,None)
        #     await d.async_set_command_mode(jog_mode,None,5)
        #     await RRN.AsyncSleep(0.01,None)

        #     # ##############################################################
        #     # if poses_list.length > 0:
        #     #     i = 0 # loop
        #     #     while i < num_loops:
        #     #         index = 0 # index in saved poses list
        #     #         while index < (poses_list.length):
        #     #             sel_pose = poses_list.options[index].value # angles as str
        #     #             joint_angles = np.fromstring(sel_pose, dtype=float, sep=',')*np.deg2rad(1) # in rad

        #     #             if not (joint_angles <= joint_upper_limits).all() or not (joint_angles >= joint_lower_limits).all():
        #     #                 print_div("Specified joints are out of range<br>")
        #     #                 return
        #     #             else:
        #     #                 try:
        #     #                     await d.async_jog_joint(joint_angles, joint_vel_limits*joint_vel_ratio, False, True, None)
        #     #                 except:
        #     #                     print_div("Specified joints might be out of range<br>")
        #     #                     return

        #     #             index += 1
        #     #         # Complete the loop, go back the first pose
        #     #         sel_pose = poses_list.options[0].value # angles as str
        #     #         joint_angles = np.fromstring(sel_pose, dtype=float, sep=',')*np.deg2rad(1) # in rad

        #     #         i += 1

        #     # else:
        #     #     print_div("Add some poses to Saved Poses and try again<br>")
        #     #     return

        # ###############################################################

        # ###############################################################

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