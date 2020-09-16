#Example Drive client in Python
from js import self as window
from js import document
from js import print_div
from js import print_div_j_info
from js import print_div_j_limit_info
from js import print_div_kin_info
from js import print_div_flag_info
from js import print_div_num_info
from js import print_div_end_info
from js import print_div_ik_info
from js import print_div_cur_pose

from RobotRaconteur.Client import *
import time
import numpy as np
import sys
import math

sys.path.append("./my_source.zip")
import general_robotics_toolbox as rox
# from qpsolvers import solve_qp

# ---------------------------BEGIN: BLOCKLY FUNCTIONS  --------------------------- #
def jog_joints2(q_i, degree_diff, is_relative):
    global is_jogging
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_jog_joints2(q_i, degree_diff, is_relative))
    else:
        print_div("Jogging has not finished yet..<br>")


async def async_jog_joints2(q_i, degree_diff, is_relative):
    global d, d_q, num_joints, joint_lower_limits, joint_upper_limits, joint_vel_limits

    # Update joint angles
    d_q = await update_joint_info() # Joint angles in radian ndarray
    
    # UPdate the end effector pose info
    pose = await update_end_info()
    
    await update_state_flags()

    if (num_joints < q_i):
        print_div("Currently Controlled Robot only have " + str(num_joints) + " joints..<br>")
    else:
        
        if (is_relative):
            joint_diff = np.zeros((num_joints,))
            joint_diff[q_i-1] = np.deg2rad(degree_diff)
        else:
            joint_diff = d_q
            joint_diff[q_i-1] = np.deg2rad(degree_diff)
        
        if not ((d_q + joint_diff) < joint_upper_limits).all() or not ((d_q + joint_diff) > joint_lower_limits).all():
            print_div("Specified joints might be out of range<br>")
        else:
            try:
                await d.async_jog_joint(joint_diff, joint_vel_limits, is_relative, True,None)
                # await RRN.AsyncSleep(2,None)
            except:
                print_div("Specified joints might be out of range2<br>")
                # import traceback
                # print_div(traceback.format_exc())
                # raise

    global is_jogging
    is_jogging = False

# ---------------------------END: BLOCKLY FUNCTIONS --------------------------- #

# ---------------------------BEGIN: GAMEPAD FUNCTIONS  --------------------------- #
def jog_joints_gamepad(joint_speed_constants):
    joint_speed_constants = np.array(joint_speed_constants,dtype="f")
    # print_div(str(joint_speed_constants)+"<br>")

    global is_jogging
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_jog_joints_gamepad(joint_speed_constants))
    # else:
    #     print_div("Jogging has not finished yet..<br>")

async def async_jog_joints_gamepad(joint_speed_constants):
    degree_diff = 1
    global d, d_q, num_joints, joint_lower_limits, joint_upper_limits, joint_vel_limits

    global is_gamepadaxisactive
    global is_gamepadbuttondown
    if (is_gamepadaxisactive or is_gamepadbuttondown): 
        # Update joint angles
        d_q = await update_joint_info() # Joint angles in radian ndarray
        
        # UPdate the end effector pose info
        pose = await update_end_info()

        await update_state_flags()


        # Trim joint speed constants accordingto number of joints
        joint_speed_constants = joint_speed_constants[:num_joints]
        # print_div("joint_speed_constants: "+str(joint_speed_constants)+"<br>")

        signs = np.divide(np.abs(joint_speed_constants),joint_speed_constants)
        np.nan_to_num(signs, copy=False)
        # print_div("signs: "+str(signs)+"<br>")

        joint_diff = np.ones((num_joints,))
        joint_diff = np.multiply(signs,np.deg2rad(degree_diff))
        # print_div("joint_diff: "+str(joint_diff)+"<br>")


        if not ((d_q + joint_diff) < joint_upper_limits).all() or not ((d_q + joint_diff) > joint_lower_limits).all():
            print_div("Specified joints might be out of range<br>")
        else:
            try:
                await d.async_jog_joint(joint_diff.astype(np.double), joint_vel_limits, True, False,None)
            except:
                print_div("Specified joints might be out of range(gamepad))")
                import traceback
                print_div(traceback.format_exc())

    global is_jogging
    is_jogging = False

def home_func_gamepad():
    # print_div('Homing...<br>')    
    global d, num_joints, joint_vel_limits
    
    d.async_jog_joint(np.zeros((num_joints,)), joint_vel_limits, False, True,None)

    global is_jogging
    is_jogging = False  
# ........................................
def jog_cartesian_gamepad(P_axis, R_axis):
    if P_axis != [0.0,0.0,0.0]:
        P_axis = np.array(P_axis,dtype="f")
        P_axis_norm = np.linalg.norm(P_axis)
        P_axis = P_axis / P_axis_norm
        np.nan_to_num(P_axis, copy=False)
    else:
        P_axis = None

    if R_axis != [0.0,0.0,0.0]:
        R_axis = np.array(R_axis,dtype="f")
        R_axis_norm = np.linalg.norm(R_axis)
        R_axis = R_axis / R_axis_norm
        np.nan_to_num(R_axis, copy=False)
    else:
        R_axis = None

    # print_div(str(P_axis)+", "+ str(R_axis) + "<br>")

    global is_jogging
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_jog_cartesian_gamepad(P_axis, R_axis))
    # else:
    #     print_div("Jogging has not finished yet..<br>")

async def async_jog_cartesian_gamepad(P_axis, R_axis):
    move_distance = 0.01 # meters
    rotate_angle = np.deg2rad(5) # radians
    
    global d, num_joints, joint_lower_limits, joint_upper_limits, joint_vel_limits
    global pose # Get the Current Pose of the robot
        
    global is_gamepadaxisactive
    global is_gamepadbuttondown
    # print_div("here 0<br>")
    if (is_gamepadaxisactive or is_gamepadbuttondown): 
        # Update joint angles
        d_q = await update_joint_info() # Joint angles in radian ndarray
        # UPdate the end effector pose info
        pose = await update_end_info()
        await update_state_flags()

        Rd = pose.R
        pd = pose.p

        # print_div("here 1<br>")
            
        if P_axis is not None:
            pd = pd + Rd.dot(move_distance * P_axis)
        if R_axis is not None:
            # R = rox.rot(np.array(([1.],[0.],[0.])), 0.261799)
            R = rox.rot(R_axis, rotate_angle)
            Rd = Rd.dot(R) # Rotate
        
        try:
            # Update desired inverse kineamtics info
            joint_angles, converged = update_ik_info(Rd,pd)
            if not converged:
                print_div("Inverse Kinematics Algo. Could not Converge<br>")
                raise
            elif not (joint_angles < joint_upper_limits).all() or not (joint_angles > joint_lower_limits).all():
                print_div("Specified joints are out of range<br>")
                raise
            else:
                await d.async_jog_joint(joint_angles, joint_vel_limits, False, True, None)
        except:
            print_div("Specified joints might be out of range<br>")

    global is_jogging
    is_jogging = False


# ---------------------------END: GAMEPAD FUNCTIONS  --------------------------- #

# ---------------------------BEGIN: JOINT SPACE JOGGING --------------------------- #
def jog_joints(q_i, sign):
    global is_mousedown
    is_mousedown = True

    global is_jogging
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_jog_joints(q_i, sign))
    else:
        print_div("Jogging has not finished yet..<br>")

async def async_jog_joints(q_i, sign):
    global plugin_jogJointSpace

    global is_mousedown
    while (is_mousedown): 
        # Call Jog Joint Space Service funtion to handle this jogging
        await plugin_jogJointSpace.async_jog_joints(q_i, sign, None)

    global is_jogging
    is_jogging = False    

def j1_pos_func(self):
    print_div('j1+ button pressed<br>')
    jog_joints(1,+1)
    # RRN.PostToThreadPool(lambda: jog_joints(1,+1))
    
def j1_neg_func(self):
    print_div('j1- button pressed<br>')
    jog_joints(1,-1)
    
def j2_pos_func(self):
    print_div('j2+ button pressed<br>')
    jog_joints(2,+1)
    #joint_vel = np.array([0.,1.,0.,0.,0.,0.,0.])
    
def j2_neg_func(self):
    print_div('j2- button pressed<br>')
    jog_joints(2,-1)
        
def j3_pos_func(self):
    print_div('j3+ button pressed<br>')
    jog_joints(3,+1)
    
def j3_neg_func(self):
    print_div('j3- button pressed<br>')
    jog_joints(3,-1)
    
def j4_pos_func(self):
    print_div('j4+ button pressed<br>')
    jog_joints(4,+1)
    
def j4_neg_func(self):
    print_div('j4- button pressed<br>')
    jog_joints(4,-1)
    
def j5_pos_func(self):
    print_div('j5+ button pressed<br>')
    jog_joints(5,+1)
    
def j5_neg_func(self):
    print_div('j5- button pressed<br>')
    jog_joints(5,-1)
    
def j6_pos_func(self):
    print_div('j6+ button pressed<br>')
    jog_joints(6,+1)
    
def j6_neg_func(self):
    print_div('j6- button pressed<br>')
    jog_joints(6,-1)

def j7_pos_func(self):
    print_div('j7+ button pressed<br>')
    jog_joints(7,+1)
    
def j7_neg_func(self):
    print_div('j7- button pressed<br>')
    jog_joints(7,-1)

# TODO: Implement this properly
def stop_func(self):
    print_div('STOP button pressed<br>')    
    global num_joints, joint_vel_limits
    #global d
    
    #d.async_jog_joint(np.zeros((num_joints,)), joint_vel_limits, False, True,None)
    global plugin_jogJointSpace
    plugin_jogJointSpace.async_jog_joints_with_limits(np.zeros((num_joints,)),np.zeros((num_joints,)),np.zeros((num_joints,)),joint_vel_limits,False,True,None)


    global is_jogging
    is_jogging = False
          

def move_to_angles_func(self):
    print_div('Move to angles button pressed<br>')
    global is_jogging
    
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_move_to_angles_func())
    else:
        print_div("Jogging has not finished yet..<br>")
    
async def async_move_to_angles_func():
    global plugin_jogJointSpace
    global num_joints, joint_vel_limits
    global is_jogging

    joint_angles = np.zeros((num_joints,))
    element_id = "j1_angle_in"
    
    for j in range(1,num_joints+1):
        element_id = "j" + str(j) + "_angle_in"
        text_container_angle = document.getElementById(element_id)
        angle = text_container_angle.value # str and in degrees
        try: # if not angle == None or not angle == "":
            joint_angles[j-1] = float(angle)* np.deg2rad(1)
        except: # else:
            print_div("Please specify angle of each joint!<br>")
            is_jogging = False
            return

    await plugin_jogJointSpace.async_jog_joints_with_limits(joint_angles,joint_angles,joint_angles,joint_vel_limits,False,True,None)

    is_jogging = False
# ---------------------------END: JOINT SPACE JOGGING --------------------------- #

# ---------------------------BEGIN: CARTESIAN SPACE JOGGING --------------------------- #
def jog_cartesian(P_axis, R_axis):
    global is_mousedown
    is_mousedown = True

    global is_jogging
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_jog_cartesian(P_axis, R_axis))
    else:
        print_div("Jogging has not finished yet..<br>")

async def async_jog_cartesian(P_axis, R_axis):
    global plugin_jogCartesianSpace

    global is_mousedown
    while (is_mousedown):
        # Call Jog Joint Space Service funtion to handle this jogging
        await plugin_jogCartesianSpace.async_jog_cartesian(P_axis, R_axis, None)

    global is_jogging
    is_jogging = False
        

def X_pos_func(self):
    print_div('X+ button pressed<br>')
    jog_cartesian(np.array(([+1.,0.,0.])), np.array(([0.,0.,0.])))
    
def X_neg_func(self):
    print_div('X- button pressed<br>')
    jog_cartesian(np.array(([-1.,0.,0.])), np.array(([0.,0.,0.])))
    
def Y_pos_func(self):
    print_div('Y+ button pressed<br>')
    jog_cartesian(np.array(([0.,+1.,0.])), np.array(([0.,0.,0.])))
    
def Y_neg_func(self):
    print_div('Y- button pressed<br>')
    jog_cartesian(np.array(([0.,-1.,0.])), np.array(([0.,0.,0.])))
    
def Z_pos_func(self):
    print_div('Z+ button pressed<br>')
    jog_cartesian(np.array(([0.,0.,+1.])), np.array(([0.,0.,0.])))
    
def Z_neg_func(self):
    print_div('Z- button pressed<br>')
    jog_cartesian(np.array(([0.,0.,-1.])), np.array(([0.,0.,0.])))

def tX_pos_func(self):
    print_div('&theta;X+ button pressed<br>')
    jog_cartesian(np.array(([0.,0.,0.])), np.array(([+1.,0.,0.])))

def tX_neg_func(self):
    print_div('&theta;X- button pressed<br>')
    jog_cartesian(np.array(([0.,0.,0.])), np.array(([-1.,0.,0.])))
    
def tY_pos_func(self):
    print_div('&theta;Y+ button pressed<br>')
    jog_cartesian(np.array(([0.,0.,0.])), np.array(([0.,+1.,0.])))
    
def tY_neg_func(self):
    print_div('&theta;Y- button pressed<br>')
    jog_cartesian(np.array(([0.,0.,0.])), np.array(([0.,-1.,0.])))
    
def tZ_pos_func(self):
    print_div('&theta;Z+ button pressed<br>')
    jog_cartesian(np.array(([0.,0.,0.])), np.array(([0.,0.,+1.])))
    
def tZ_neg_func(self):
    print_div('&theta;Z- button pressed<br>')
    jog_cartesian(np.array(([0.,0.,0.])), np.array(([0.,0.,-1.])))


def update_ik_info(R_d, p_d):
    # R_d, p_d: Desired orientation and position
    global robot
    global d_q # Get Current Joint angles in radian ndarray 
    global num_joints
    
    q_cur = d_q # initial guess on the current joint angles
    q_cur = q_cur.reshape((num_joints,1)) 
    
    epsilon = 0.001 # Damping Constant
    Kq = epsilon * np.eye(len(q_cur)) # small value to make sure positive definite used in Damped Least Square
    # print_div( "<br> Kq " + str(Kq) ) # DEBUG
    
    max_steps = 200 # number of steps to for convergence
    
    # print_div( "<br> q_cur " + str(q_cur) ) # DEBUG
    
    itr = 0 # Iterations
    converged = False
    while itr < max_steps and not converged:
    
        pose = rox.fwdkin(robot,q_cur)
        R_cur = pose.R
        p_cur = pose.p
        
        #calculate current Jacobian
        J0T = rox.robotjacobian(robot,q_cur)
        
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
        
        q_cur = q_cur - delta.reshape((num_joints,1))
        
        # Convergence Check
        converged = (np.hstack((s,EP)) < 0.0001).all()
        # print_div( "<br> converged? " + str(converged) ) # DEBUG
        
        itr += 1 # Increase the iteration
    
    joints_text=""
    for i in q_cur:
        joints_text+= "(%.3f, %.3f) " % (np.rad2deg(i), i)   
    print_div_ik_info(str(rox.Transform(R_d,p_d)) +"<br>"+ joints_text +"<br>"+ str(converged) + ", itr = " + str(itr))
    return q_cur, converged
#########################################################
# ZYX Euler angles calculation from rotation matrix
def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)

def euler_angles_from_rotation_matrix(R):
    '''
    From a paper by Gregory G. Slabaugh (undated),
    "Computing Euler angles from a rotation matrix
    '''
    phi = 0.0
    if isclose(R[2,0],-1.0):
        theta = math.pi/2.0
        psi = math.atan2(R[0,1],R[0,2])
    elif isclose(R[2,0],1.0):
        theta = -math.pi/2.0
        psi = math.atan2(-R[0,1],-R[0,2])
    else:
        theta = -math.asin(R[2,0])
        cos_theta = math.cos(theta)
        psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return psi, theta, phi #  x y z for Rz * Ry * Rx
#########################################################
#########################################################

async def update_end_info():
    global robot
    global d_q
    
    pose = rox.fwdkin(robot, d_q)
       
    print_div_end_info(str(pose))
    # print_div_end_info( np.array_str(pose.R, precision=4, suppress_small=True).replace('\n', '\n' + ' '*4))
    
    # Calculate euler ZYX angles from pose and write them into:
    x,y,z = euler_angles_from_rotation_matrix(pose.R)
    str_rx = "%.2f" % (np.rad2deg(x))
    str_ry = "%.2f" % (np.rad2deg(y))
    str_rz = "%.2f" % (np.rad2deg(z))
    str_px = "%.3f" % (pose.p[0])
    str_py = "%.3f" % (pose.p[1])
    str_pz = "%.3f" % (pose.p[2])
    xyz_orient_str = "[" + str_rx + ", " + str_ry + ", " + str_rz + "]"
    xyz_positi_str = "[" + str_px + ", " + str_py + ", " + str_pz + "]"
    print_div_cur_pose(xyz_orient_str,xyz_positi_str)
    return pose
# ---------------------------END: CARTESIAN SPACE JOGGING --------------------------- #

# ---------------------------BEGIN: SAVE PLAYBACK POSES --------------------------- #
def save_cur_pose_func(self):
    print_div('Saving to "Saved Poses" list..<br>')
    
    global d_q # Get the current joint angles in rad ndarray
    # Convert them into degrees for text view
    joints_text=""
    for i in d_q:
        joints_text+= "%.2f," % (np.rad2deg(i))
    joints_text = joints_text[:-1] # Delete the last comma

    # Add the current joint angles to the saved poses list
    element_id = "saved_poses_list"
    poses_list = document.getElementById(element_id)
    option = document.createElement("option")
    option.text = joints_text
    poses_list.add(option)

def go_sel_pose_func(self):
    print_div("Moving to selected pose..<br>")

    global is_jogging

    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_go_sel_pose_func())
    else:
        print_div("Jogging has not finished yet..<br>")


async def async_go_sel_pose_func():
    global d, num_joints, joint_lower_limits, joint_upper_limits, joint_vel_limits
    global is_jogging

    # Read the selected pose from the browser
    element_id = "saved_poses_list"
    poses_list = document.getElementById(element_id)
    index = poses_list.selectedIndex

    try:
        if index == -1:
            print_div("Please select a pose from Saved Poses.<br>")
            raise
        else:
            sel_pose = poses_list.options[index].value # angles as str
            joint_angles = np.fromstring(sel_pose, dtype=float, sep=',')*np.deg2rad(1) # in rad

            if not (joint_angles < joint_upper_limits).all() or not (joint_angles > joint_lower_limits).all():
                print_div("Specified joints are out of range<br>")
                raise
            else:
                try:
                    await d.async_jog_joint(joint_angles, joint_vel_limits, False, True,None)
                except:
                    print_div("Specified joints might be out of range<br>")
                    raise
    except:
        is_jogging = False
    is_jogging = False

def playback_poses_func(self):
    print_div("Playing Back Poses..")
    loop.call_soon(async_playback_poses_func())

async def async_playback_poses_func():
    global d, num_joints, joint_lower_limits, joint_upper_limits, joint_vel_limits
    global JointTrajectoryWaypoint, JointTrajectory
    # Get Joint Names
    robot_info = await d.async_get_robot_info(None)
    joint_names = [j.joint_identifier.name for j in robot_info.joint_info]

    # Get elements, poses and paramaters from web interface
    poses_list = document.getElementById("saved_poses_list")
    num_loops_elem = document.getElementById("num_loops_in")
    num_loops = int(num_loops_elem.value)
    joint_vel_range = document.getElementById("joint_vel_range")
    joint_vel_ratio = float(joint_vel_range.value)/100.0

    time_loops_elem = document.getElementById("time_loops_in")
    time_loops = float(time_loops_elem.value)

    # Time to complete the playback
    t_complete = time_loops #seconds
    
    # Create waypoints array 
    waypoints = []
    if poses_list.length >= 4:
        index = 0 # index in saved poses list
        while index < (poses_list.length):
            sel_pose = poses_list.options[index].value # angles as str
            joint_angles = np.fromstring(sel_pose, dtype=float, sep=',')*np.deg2rad(1) # in rad

            if not (joint_angles < joint_upper_limits).all() or not (joint_angles > joint_lower_limits).all():
                print_div("Specified joints are out of range<br>")
                return
            else:
                # Go to initial waypoint
                if index == 0:
                    await d.async_jog_joint(joint_angles, joint_vel_limits, False, True,None)
                    await RRN.AsyncSleep(2,None)

                # Define the time to be at that waypoint
                t = float(index)*(float(t_complete)/float(poses_list.length))
                # Add the joint angles to waypoints array
                wp = JointTrajectoryWaypoint()
                wp.joint_position = joint_angles # (j_end - j_start)*(float(i)/10.0) + j_start
                wp.time_from_start = t
                waypoints.append(wp)

            index += 1
        # Complete the loop, Add the 1st joint angles to waypoints array again
        t = float(index)*(float(t_complete)/float(poses_list.length))
        # Add the joint angles to waypoints array
        wp = JointTrajectoryWaypoint()
        wp.joint_position = waypoints[0].joint_position
        wp.time_from_start = t
        waypoints.append(wp)

    else:
        print_div("You need at least 4 different points. Add some poses to Saved Poses and try again<br>")
        # # Put robot to jogging mode back
        # await d.async_set_command_mode(halt_mode,None,5)
        # await RRN.AsyncSleep(0.01,None)
        # await d.async_set_command_mode(jog_mode,None,5)
        # await RRN.AsyncSleep(0.01,None)
        return

    # Put robot to trajectory mode
    await d.async_set_command_mode(halt_mode,None,5)
    await RRN.AsyncSleep(0.01,None)
    await d.async_set_command_mode(trajectory_mode,None,5)
    await RRN.AsyncSleep(0.01,None)

    # Create the trajectory
    traj = JointTrajectory()
    traj.joint_names = joint_names
    traj.waypoints = waypoints
    d.async_set_speed_ratio(joint_vel_ratio,None,5)

    try:
        # Execute the trajectory number of loops times
        i = 0 # loop
        while i < num_loops:
            traj_gen = await d.async_execute_trajectory(traj, None)
        
            while (True):
                t = time.time()

                # robot_state = state_w.InValue
                try:
                    # print_div("Here")
                    res = await traj_gen.AsyncNext(None, None)        
                    # print_div(res.action_status)
                except RR.StopIterationException:
                    break

            i += 1
            print_div("Loop:" + str(i) + "is done..<br>")
    except:
        # import traceback
        # print_div(traceback.format_exc())
        print_div("Robot accelaration or velocity limits might be out of range. Increase the loop time or slow down the speed ratio<br>")
        # return
        # raise

    # Put robot to jogging mode back
    await d.async_set_command_mode(halt_mode,None,5)
    await RRN.AsyncSleep(0.01,None)
    await d.async_set_command_mode(jog_mode,None,5)
    await RRN.AsyncSleep(0.01,None)

    # ##############################################################
    # if poses_list.length > 0:
    #     i = 0 # loop
    #     while i < num_loops:
    #         index = 0 # index in saved poses list
    #         while index < (poses_list.length):
    #             sel_pose = poses_list.options[index].value # angles as str
    #             joint_angles = np.fromstring(sel_pose, dtype=float, sep=',')*np.deg2rad(1) # in rad

    #             if not (joint_angles <= joint_upper_limits).all() or not (joint_angles >= joint_lower_limits).all():
    #                 print_div("Specified joints are out of range<br>")
    #                 return
    #             else:
    #                 try:
    #                     await d.async_jog_joint(joint_angles, joint_vel_limits*joint_vel_ratio, False, True, None)
    #                 except:
    #                     print_div("Specified joints might be out of range<br>")
    #                     return

    #             index += 1
    #         # Complete the loop, go back the first pose
    #         sel_pose = poses_list.options[0].value # angles as str
    #         joint_angles = np.fromstring(sel_pose, dtype=float, sep=',')*np.deg2rad(1) # in rad

    #         i += 1

    # else:
    #     print_div("Add some poses to Saved Poses and try again<br>")
    #     return

###############################################################
# ---------------------------END: SAVE PLAYBACK POSES --------------------------- #

async def update_state_flags():
    # For reading robot state flags
    #print_div_flag_info("State Flags Updating..")
    
    global d
    d_state = await d.robot_state.AsyncPeekInValue(None,5)
    
    global robot_const
    state_flags_enum = robot_const['RobotStateFlags']
    # print_div(state_flags_enum.items())
    
    flags_text =""
    for flag_name, flag_code in state_flags_enum.items():
        if flag_code & d_state[0].robot_state_flags != 0:
            flags_text += flag_name + " "
    print_div_flag_info(flags_text)
  
  
async def update_joint_info():
    # For reading Joint Angles
    # print_div_j_info("Joint info Updating..")
    
    global d
    d_state = await d.robot_state.AsyncPeekInValue(None,5)    
    d_q = d_state[0].joint_position
    
    joints_text=""
    for i in d_q:
        # joints_text+= "%.2f, %.2f<br>" % (np.rad2deg(i), i)
        joints_text+= "%.2f<br>" % (np.rad2deg(i))
    # joints_text += ", shape = " + str(d_q.shape) + "<br>" # DEBUG
    print_div_j_info(joints_text)
    # print_div_j_info(type(d_q).__name__)
    # print_div_j_info(d_q.tolist())
    return d_q # in radian ndarray

async def update_num_info():
    # For reading about number of robot joints, joint types, joint limits etc
    # print_div_num_info("Number of Joints info updating")
 
    global d    
    robot_info = await d.async_get_robot_info(None) 
    joint_info = robot_info.joint_info # A list of jointInfo
    
    joint_types = [] # A list or array of N numbers containing the joint type. 1 for rotary, 3 for prismatic
    joint_lower_limits = [] # list or numpy.array
    joint_upper_limits = [] # list or numpy.array
    joint_vel_limits = [] # list or numpy.array
    joint_acc_limits = [] # list or numpy.array
    joint_names = [] # list of string
    joint_uuids = [] 
    for joint in joint_info:
        joint_types.append(joint.joint_type)
        joint_lower_limits.append(joint.joint_limits.lower)
        joint_upper_limits.append(joint.joint_limits.upper)
        joint_vel_limits.append(joint.joint_limits.velocity)
        joint_acc_limits.append(joint.joint_limits.acceleration)
        joint_names.append(joint.joint_identifier.name)
        joint_uuids.append(joint.joint_identifier.uuid)
        
    # convert them to numpy arrays
    joint_types = np.asarray(joint_types)
    joint_lower_limits = np.asarray(joint_lower_limits)
    joint_upper_limits = np.asarray(joint_upper_limits)
    joint_vel_limits = np.asarray(joint_vel_limits)
    joint_acc_limits = np.asarray(joint_acc_limits)
    
    joint_lower_limits_text = ""    
    for i in joint_lower_limits:
        joint_lower_limits_text += "%.2f " % (np.rad2deg(i))
        
    joint_upper_limits_text = ""
    for i in joint_upper_limits:
        joint_upper_limits_text += "%.2f " % (np.rad2deg(i))
    
    print_div_j_limit_info(joint_lower_limits_text, joint_upper_limits_text)
    
    print_div_num_info( str(len(joint_info)) +"<br>"+ str(joint_types) +"<br>"+ str(joint_vel_limits) +"<br>"+ str(joint_acc_limits) +"<br>"+ str(joint_names) ) # +"<br>"+ str(joint_uuids))
    
    return len(joint_info), joint_types, joint_lower_limits, joint_upper_limits, joint_vel_limits, joint_acc_limits, joint_names
     
async def update_kin_info():
    global d_q # Joint angles in radian ndarray
    global d
    global num_joints    
    robot_info = await d.async_get_robot_info(None)
    chains = robot_info.chains # Get RobotKinChainInfo
    H = chains[0].H # Axes of the joints, 3xN 
    P = chains[0].P # P vectors between joint centers (Product of Exponenetials Convention)
    # print_div_kin_info(type(H).__name__)
    
    H_shaped = np.zeros((3, num_joints))
    H_text = "H (axis):<br>"
    itr = 0
    for i in H:
        H_shaped[:,itr] = (i[0],i[1],i[2])
        H_text+= "|" + " %.1f " %i[0] + " %.1f " %i[1] + " %.1f " %i[2]  + "|<br> "
        itr += 1
    
    itr = 0
    P_shaped = np.zeros((3, num_joints+1))    
    P_text = "P (in meters):<br>"
    for i in P:
        P_shaped[:,itr] = (i[0],i[1],i[2])
        P_text+= "|" + " %.3f " %i[0] + " %.3f " %i[1] + " %.3f " %i[2]  + " |<br> "
        itr += 1
        
    #print_div_j_info(joints_text)
    print_div_kin_info(H_text + "<br>" + P_text)  
    return H_shaped, P_shaped  


def mouseup_func(self):
    global is_mousedown
    is_mousedown = False
    # print_div("Mouse is up<br>")


def gamepadbuttonup():
    global is_gamepadbuttondown
    is_gamepadbuttondown = False
    # print_div("Gamepadbutton is up<br>")

def gamepadbuttondown():
    global is_gamepadbuttondown
    is_gamepadbuttondown = True
    # print_div("Gamepadbutton is down<br>")

def gamepadaxisinactive():
    global is_gamepadaxisactive
    is_gamepadaxisactive = False
    # print_div("Gamepadaxis is inactive<br>")

def gamepadaxisactive():
    global is_gamepadaxisactive
    is_gamepadaxisactive = True
    # print_div("Gamepadaxis is active<br>")



async def client_drive():
    # rr+ws : WebSocket connection without encryption
    ip = '192.168.50.152'

    # url ='rr+ws://localhost:58653?service=sawyer'    
    url ='rr+ws://'+ ip +':58653?service=sawyer'   
    # url ='rr+ws://128.113.224.23:58654?service=sawyer' # sawyer in lab

    # url ='rr+ws://localhost:58655?service=robot' #ABB
    # url ='rr+ws://192.168.50.118:58655?service=robot' #ABB

    # url = 'rr+ws://localhost:23333?service=robot' # Dr.Wasons's Robot

    print_div('Program started, please wait..<br>')

    try:
        #Connect to the service
        global d # d is the robot object from RR
        d = await RRN.AsyncConnectService(url,None,None,None,None)
        d.async_reset_errors(None)
        d.async_enable(None)
        
        # Define Robot modes
        global robot_const, halt_mode, jog_mode, position_mode, trajectory_mode
        robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", d)
        halt_mode = robot_const["RobotCommandMode"]["halt"]
        jog_mode = robot_const["RobotCommandMode"]["jog"]
        position_mode = robot_const["RobotCommandMode"]["velocity_command"]
        trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]
        
        # Import Necessary Structures
        global JointTrajectoryWaypoint, JointTrajectory
        JointTrajectoryWaypoint = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectoryWaypoint",d)
        JointTrajectory = RRN.GetStructureType("com.robotraconteur.robotics.trajectory.JointTrajectory",d)


        c_services = await RRN.AsyncConnectService('rr+tcp://' + ip + ':port?service=RobotRaconteurServiceIndex',None,None,None,None)
        services = await c_services.async_GetLocalNodeServices(None)
        print_div('Available services:<br>')
        for s in services.items():
            print_div(str(s) +"<br>")


        # Put robot to jogging mode
        # await d.async_set_command_mode(halt_mode,None,5)
        # await RRN.AsyncSleep(0.1,None)
        # await d.async_set_command_mode(jog_mode,None,5)
        # await RRN.AsyncSleep(0.1,None)

        # # Put the robot to velocity conrol mode
        # await d.async_set_command_mode(halt_mode,None,5)
        # await RRN.AsyncSleep(1,None)
        # await d.async_set_command_mode(position_mode,None,5)
        # await RRN.AsyncSleep(1,None)

        # # Put robot to trajectory mode
        # await d.async_set_command_mode(halt_mode,None,5)
        # await RRN.AsyncSleep(0.1,None)
        # await d.async_set_command_mode(trajectory_mode,None,5)
        # await RRN.AsyncSleep(0.1,None)

        # PLUGIN SERVICE CONNECTIONS BEGIN________________________________

        ## JogJointSpace plugin
        print_div('JogJointSpace plugin is connecting..<br>')

        # url_plugin_jogJointSpace = 'rr+ws://localhost:8890?service=JogJointSpace'
        url_plugin_jogJointSpace = 'rr+ws://' + ip + ':8890?service=JogJointSpace'
        global plugin_jogJointSpace
        plugin_jogJointSpace = await RRN.AsyncConnectService(url_plugin_jogJointSpace,None,None,None,None)
        await plugin_jogJointSpace.async_connect2robot(url,None)

        print_div('JogJointSpace plugin is connected..<br>')

        ## JogCartesianSpace plugin
        print_div('JogCartesianSpace plugin is connecting..<br>')

        # url_plugin_jogCartesianSpace = 'rr+ws://localhost:8891?service=JogCartesianSpace'
        url_plugin_jogCartesianSpace = 'rr+ws://' + ip + ':8891?service=JogCartesianSpace'
        global plugin_jogCartesianSpace
        plugin_jogCartesianSpace = await RRN.AsyncConnectService(url_plugin_jogCartesianSpace,None,None,None,None)
        await plugin_jogCartesianSpace.async_connect2robot(url,None)

        print_div('JogJointSpace plugin is connected..<br>')
        
        ## Vision plugin
        print_div('Vision plugin is connecting..<br>')

        # url_plugin_vision = 'rr+ws://localhost:8889?service=Vision'
        url_plugin_vision = 'rr+ws://' + ip + ':8889?service=Vision'
        global plugin_vision
        plugin_vision = await RRN.AsyncConnectService(url_plugin_vision,None,None,None,None)

        print_div('Vision plugin is connected!<br>')

        # PLUGIN SERVICE CONNECTIONS END__________________________________
         
        print_div('READY!<br>')
        
        global d_q
        # Get the current joint positions
        d_q = await update_joint_info() # Joint angles in radian ndarray, N x 1
                  
        global num_joints, joint_types, joint_lower_limits, joint_upper_limits, joint_vel_limits, joint_acc_limits, joint_names
        # Get the number of Joints, Joint Types, Limits etc in the robot.
        num_joints, joint_types, joint_lower_limits, joint_upper_limits, joint_vel_limits, joint_acc_limits, joint_names  = await update_num_info()
        
        global H, P
        # Get the kinematics info, P and H in product of exponentials convention
        H, P = await update_kin_info()
        
        # print_div_end_info(str(H))
        
        global robot # Robotics Toolbox Robot Object (NOT THE RR ROBOT OBJECT, IT IS d)
        # Now we are ready to create the robot from toolbox
        # robot = rox.Robot(H,P,joint_types-1,joint_lower_limits,joint_upper_limits,joint_vel_limits,joint_acc_limits)
        robot = rox.Robot(H,P,joint_types-1)
        
        global pose # Current pose object from Robotics Toolbox of the end effector
        # UPdate the end effector pose info
        pose = await update_end_info()
            
        # Element references
        # Joint Space Control Buttons
        button_stop = document.getElementById("stop_btn")
        
        button_j1_pos = document.getElementById("j1_pos_btn")
        button_j1_neg = document.getElementById("j1_neg_btn")
        
        button_j2_pos = document.getElementById("j2_pos_btn")
        button_j2_neg = document.getElementById("j2_neg_btn")
        
        button_j3_pos = document.getElementById("j3_pos_btn")
        button_j3_neg = document.getElementById("j3_neg_btn")
        
        button_j4_pos = document.getElementById("j4_pos_btn")
        button_j4_neg = document.getElementById("j4_neg_btn")
        
        button_j5_pos = document.getElementById("j5_pos_btn")
        button_j5_neg = document.getElementById("j5_neg_btn")
        
        button_j6_pos = document.getElementById("j6_pos_btn")
        button_j6_neg = document.getElementById("j6_neg_btn")
        
        button_j7_pos = document.getElementById("j7_pos_btn")
        button_j7_neg = document.getElementById("j7_neg_btn")
        
        button_angles_submit = document.getElementById("j_angles_submit_btn")
        
        # Task Space Control Buttons
        button_X_pos = document.getElementById("X_pos_btn")
        button_X_neg = document.getElementById("X_neg_btn")
        
        button_Y_pos = document.getElementById("Y_pos_btn")
        button_Y_neg = document.getElementById("Y_neg_btn")
        
        button_Z_pos = document.getElementById("Z_pos_btn")
        button_Z_neg = document.getElementById("Z_neg_btn")
        
        button_theta_X_pos = document.getElementById("theta_X_pos_btn") 
        button_theta_X_neg = document.getElementById("theta_X_neg_btn")
        
        button_theta_Y_pos = document.getElementById("theta_Y_pos_btn")
        button_theta_Y_neg = document.getElementById("theta_Y_neg_btn")
        
        button_theta_Z_pos = document.getElementById("theta_Z_pos_btn")
        button_theta_Z_neg = document.getElementById("theta_Z_neg_btn")
        
        # Move robot to a certain default position
        # move_to_angles_func(None)

        # Playback Poses Buttons
        button_save_cur_pose = document.getElementById("save_pose_btn")
        button_go_sel_pose = document.getElementById("go_sel_pose_btn")
        button_playback_poses = document.getElementById("playback_poses_btn")


        button_stop.addEventListener("click", stop_func)
        button_j1_pos.addEventListener("mousedown", j1_pos_func)
        button_j1_neg.addEventListener("mousedown", j1_neg_func)
        
        button_j2_pos.addEventListener("mousedown", j2_pos_func)
        button_j2_neg.addEventListener("mousedown", j2_neg_func)
                   
        button_j3_pos.addEventListener("mousedown", j3_pos_func)
        button_j3_neg.addEventListener("mousedown", j3_neg_func)
                 
        button_j4_pos.addEventListener("mousedown", j4_pos_func)
        button_j4_neg.addEventListener("mousedown", j4_neg_func)
           
        button_j5_pos.addEventListener("mousedown", j5_pos_func)
        button_j5_neg.addEventListener("mousedown", j5_neg_func)
                   
        button_j6_pos.addEventListener("mousedown", j6_pos_func)
        button_j6_neg.addEventListener("mousedown", j6_neg_func)
                    
        button_j7_pos.addEventListener("mousedown", j7_pos_func) 
        button_j7_neg.addEventListener("mousedown", j7_neg_func)
        
        button_angles_submit.addEventListener("click", move_to_angles_func)
        
        button_X_pos.addEventListener("mousedown", X_pos_func)
        button_X_neg.addEventListener("mousedown", X_neg_func)
        
        button_Y_pos.addEventListener("mousedown", Y_pos_func)
        button_Y_neg.addEventListener("mousedown", Y_neg_func)
        
        button_Z_pos.addEventListener("mousedown", Z_pos_func)
        button_Z_neg.addEventListener("mousedown", Z_neg_func)
        
        button_theta_X_pos.addEventListener("mousedown", tX_pos_func)
        button_theta_X_neg.addEventListener("mousedown", tX_neg_func)
        
        button_theta_Y_pos.addEventListener("mousedown", tY_pos_func)
        button_theta_Y_neg.addEventListener("mousedown", tY_neg_func)
        
        button_theta_Z_pos.addEventListener("mousedown", tZ_pos_func)
        button_theta_Z_neg.addEventListener("mousedown", tZ_neg_func)
        
        button_save_cur_pose.addEventListener("click", save_cur_pose_func)
        button_go_sel_pose.addEventListener("click", go_sel_pose_func)
        button_playback_poses.addEventListener("click", playback_poses_func)

        global is_jogging
        is_jogging = False

        global is_mousedown
        is_mousedown = False        

        document.addEventListener("mouseup", mouseup_func)

        global is_gamepadbuttondown
        is_gamepadbuttondown = False

        global is_gamepadaxisactive
        is_gamepadaxisactive = False
        

        while True:
            
            # Update joint angles
            d_q = await update_joint_info() # Joint angles in radian ndarray
            
            # UPdate the end effector pose info
            pose = await update_end_info()
            
            await update_state_flags()
            

            
    except:
        import traceback
        print_div(traceback.format_exc())
        raise

loop = RR.WebLoop()
loop.call_soon(client_drive())

# RR.WebLoop.run(client_drive())
