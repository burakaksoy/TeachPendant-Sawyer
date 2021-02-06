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

from js import Blockly


from RobotRaconteur.Client import *
# import time
import numpy as np
import sys
# import math

# sys.path.append("./my_source.zip")
# import general_robotics_toolbox as rox

# # ---------------------------BEGIN: BLOCKLY FUNCTIONS  --------------------------- #
def execute_blockly_func(self):
    print_div("Blockly Execute button is clicked!<br><br>")
    workspace = Blockly.getMainWorkspace()
    code_text = Blockly.Python.workspaceToCode(workspace)

    # # Encode the block tree as XML Dom
    # workspace_xml = Blockly.Xml.workspaceToDom(workspace)
    # # Convert XML Dom to string representation (ugly)
    # workspace_xml_str = Blockly.Xml.domToText(workspace_xml)
    # # Convert XML Dom to string representation (pretty)
    # workspace_xml_pretty_str = Blockly.Xml.domToPrettyText(workspace_xml)

    # print_div("<br>workspace_xml:<br>")
    # print_div(workspace_xml)
    # print_div("<br>workspace_xml_str:<br>")
    # print_div(workspace_xml_str)
    # print_div("<br>workspace_xml_pretty_str:<br>")
    # print_div(workspace_xml_pretty_str)
    

    # print_div(code_text)

    global is_jogging
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_execute_blockly(code_text))
    else:
        print_div("Blockly execution has not finished yet..<br>")

    
async def async_execute_blockly(code_text):
    global plugin_blockly
    output = await plugin_blockly.async_execute_blockly(code_text, None)
    print_div(output) # print the output of the execution
    
    print_div("<br>Blockly execution is completed.<br>") # Print a blank space after ecexution
    global is_jogging
    is_jogging = False

async def async_update_blockly_saved_workspaces():
    global plugin_blockly
    workspace_files_lst = await plugin_blockly.async_blockly_saved_workspaces(None)

    element_id = "saved_blocks_list"
    blocks_list = document.getElementById(element_id)
    
    # Clear the UI saved workspaces list before the update
    length = blocks_list.options.length
    i = length - 1
    while i >= 0:
        blocks_list.remove(i)
        i -= 1

    # Update the UI list with the new elements
    for file_name in workspace_files_lst:
        option = document.createElement("option")
        option.text = file_name
        blocks_list.add(option)

def load_blocks_func(self):
    print_div("Load Blocks button is clicked!<br>")
    loop.call_soon(async_load_blocks())

async def async_load_blocks():
    workspace = Blockly.getMainWorkspace() # get the Blockly workspace

    # Read the selected pose index from the browser
    element_id = "saved_blocks_list"
    blocks_list = document.getElementById(element_id)
    index = blocks_list.selectedIndex
    try:
        if index == -1:
            print_div("Please select a file from Saved Blocks.<br>")
        else:
            file_name = blocks_list.options[index].value
            global plugin_blockly
            xml_text = await plugin_blockly.async_blockly_load_workspace(file_name,None)
            xml = Blockly.Xml.textToDom(xml_text)
            Blockly.Xml.domToWorkspace(xml, workspace)
    except:
        import traceback
        print_div(traceback.format_exc())
        # raise
        pass

    # Update the available saved workspaces list
    await async_update_blockly_saved_workspaces()

def delete_blocks_func(self):
    print_div("Delete Blocks button is clicked!<br>")
    loop.call_soon(async_delete_blocks())

async def async_delete_blocks():
    # Read the selected pose index from the browser
    element_id = "saved_blocks_list"
    blocks_list = document.getElementById(element_id)
    index = blocks_list.selectedIndex
    try:
        if index == -1:
            print_div("Please select a file from Saved Blocks.<br>")
        else:
            file_name = blocks_list.options[index].value
            # Ask user: are you sure you want to delete 'file_name'
            resp = window.confirm("Sure you want to delete '" + file_name + "'?")
            # if sure delete, else ignore and return
            if resp:
                global plugin_blockly
                await plugin_blockly.async_blockly_delete_workspace(file_name,None)
    except:
        import traceback
        print_div(traceback.format_exc())
        # raise
        pass

    # Update the available saved workspaces list
    await async_update_blockly_saved_workspaces()


def edit_blocks_name_func(self):
        print_div("Edit Blocks' Name button is clicked!<br>")
        loop.call_soon(async_edit_blocks_name())

async def async_edit_blocks_name():
    # Read the selected pose index from the browser
    element_id = "saved_blocks_list"
    blocks_list = document.getElementById(element_id)
    index = blocks_list.selectedIndex
    try:
        if index == -1:
            print_div("Please select a file from Saved Blocks.<br>")
        else:
            file_name = blocks_list.options[index].value
            # Ask user: New name for 'file_name'
            file_name_new = window.prompt("Please enter the new file name (Don't forget .xml file extension)", file_name);
            if (file_name_new != None and file_name_new != ""):
                global plugin_blockly
                await plugin_blockly.async_blockly_edit_workspace_name(file_name, file_name_new, None)
    except:
        import traceback
        print_div(traceback.format_exc())
        # raise
        pass

    # Update the available saved workspaces list
    await async_update_blockly_saved_workspaces()

def save_blocks_func(self):
        print_div("Save Blocks button is clicked!<br>")
        loop.call_soon(async_save_blocks())

async def async_save_blocks():
    # Read the selected pose index from the browser
    element_id = "saved_blocks_list"
    blocks_list = document.getElementById(element_id)
    index = blocks_list.selectedIndex
    try:
        if index == -1:
            print_div("Please select a file from Saved Blocks.<br>")
        else:
            file_name = blocks_list.options[index].value
            # Ask user: Sure you want to overwrite "file_name"
            resp = window.confirm("Sure you want to overwrite '" + file_name + "'?")
            # if sure overwrite, else ignore and return
            if resp:
                workspace = Blockly.getMainWorkspace() # get the Blockly workspace
                # Encode the block tree as XML Dom
                workspace_xml = Blockly.Xml.workspaceToDom(workspace)
                # Convert XML Dom to string representation (pretty)
                workspace_xml_pretty_str = Blockly.Xml.domToPrettyText(workspace_xml)

                global plugin_blockly
                await plugin_blockly.async_blockly_save_workspace(file_name,workspace_xml_pretty_str,None)
    except:
        import traceback
        print_div(traceback.format_exc())
        # raise
        pass

    # Update the available saved workspaces list
    await async_update_blockly_saved_workspaces()


def save_as_blocks_func(self):
    print_div("Save Blocks As button is clicked!<br>")
    loop.call_soon(async_save_as_blocks())

async def async_search_blockly_saved_workspaces(filename):
    element_id = "saved_blocks_list"
    blocks_list = document.getElementById(element_id)
    length = blocks_list.options.length  # Number of existing files
    i = length - 1
    while i >= 0:
        if blocks_list.options[i].value == filename:
            return True # filename is found in the options
        i -= 1

    return False

async def async_save_as_blocks():
    try:
        # Ask user: New name for 'file_name'
        file_name_new = window.prompt("Please enter the new file name to save workspace (Don't forget .xml file extension)", "new_workspace.xml");
        if (file_name_new != None and file_name_new != ""):
            # Search the existing filenames to check for coincidence
            is_found = await async_search_blockly_saved_workspaces(file_name_new)

            if not is_found:
                workspace = Blockly.getMainWorkspace() # get the Blockly workspace
                # Encode the block tree as XML Dom
                workspace_xml = Blockly.Xml.workspaceToDom(workspace)
                # Convert XML Dom to string representation (pretty)
                workspace_xml_pretty_str = Blockly.Xml.domToPrettyText(workspace_xml)

                global plugin_blockly
                await plugin_blockly.async_blockly_save_workspace_as(file_name_new, workspace_xml_pretty_str, None)
            else:
                # print_div("The file already exists! Try another file name or use Save button to save on an existing file<br>")
                window.alert("The file already exists! Try another file name or use SAVE button to save on an existing file")
    except:
        import traceback
        print_div(traceback.format_exc())
        # raise
        pass

    # Update the available saved workspaces list
    await async_update_blockly_saved_workspaces()

def clear_blocks_func(self):
        print_div("Clear Blocks button is clicked!<br>")
        loop.call_soon(async_clear_blocks())

async def async_clear_blocks():
    # Ask user: Sure you want to clear the workspace 
    resp = window.confirm("Sure you want to clear the current workspace?")
    # if sure overwrite, else ignore and return
    if resp:
        workspace = Blockly.getMainWorkspace() # get the Blockly workspace
        workspace.clear() # Dispose of all blocks and comments in workspace.

    # Update the available saved workspaces list
    await async_update_blockly_saved_workspaces()

# # ---------------------------END: BLOCKLY FUNCTIONS --------------------------- #

# ---------------------------BEGIN: GAMEPAD FUNCTIONS  --------------------------- #
def jog_joints_gamepad(joint_speed_constants):
    joint_speed_constants = np.array(joint_speed_constants,dtype="f")
    # print_div(str(joint_speed_constants)+"<br>")

    global is_jogging
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_jog_joints_gamepad(joint_speed_constants))
    else:
        print_div("Jogging has not finished yet..<br>")

async def async_jog_joints_gamepad(joint_speed_constants):
    # degree_diff = 1
    # global d, num_joints, joint_lower_limits, joint_upper_limits, joint_vel_limits

    global plugin_jogJointSpace
    global is_gamepadaxisactive
    global is_gamepadbuttondown
    if (is_gamepadaxisactive or is_gamepadbuttondown): 
        # # Update joint angles
        # d_q, _ = await update_joint_info() # Joint angles in radian ndarray, N x 1
        
        # # Trim joint speed constants accordingto number of joints
        # joint_speed_constants = joint_speed_constants[:num_joints]
        # # print_div("joint_speed_constants: "+str(joint_speed_constants)+"<br>")

        # signs = np.divide(np.abs(joint_speed_constants),joint_speed_constants)
        # np.nan_to_num(signs, copy=False)
        # # print_div("signs: "+str(signs)+"<br>")

        # joint_diff = np.ones((num_joints,))
        # joint_diff = np.multiply(signs,np.deg2rad(degree_diff))
        # print_div("joint_diff: "+str(joint_diff)+"<br>")

        # if not ((d_q + joint_diff) < joint_upper_limits).all() or not ((d_q + joint_diff) > joint_lower_limits).all():
        #     print_div("Specified joints might be out of range<br>")
        # else:
        #     try:
        #         # await d.async_jog_joint(joint_diff.astype(np.double), joint_vel_limits, True, False,None)
        #         await d.async_jog_freespace((d_q + joint_diff).astype(np.double), joint_vel_limits, False,None)
        #     except:
        #         print_div("Specified joints might be out of range(gamepad))")
        #         import traceback
        #         print_div(traceback.format_exc())


        # Call Jog Joint Space Service funtion to handle this jogging
        await plugin_jogJointSpace.async_jog_joints_gamepad(joint_speed_constants, None)
        # TODO above line

    global is_jogging
    is_jogging = False

def home_func_gamepad():
    # print_div('Homing...<br>')    
    global plugin_jogJointSpace
    plugin_jogJointSpace.async_jog_joints_zeros(None)

    global is_jogging
    is_jogging = False 
# ........................................
def jog_cartesian_gamepad(P_axis, R_axis):
    if P_axis != [0.0,0.0,0.0]:
        P_axis = np.array(P_axis,dtype=np.dtype('d'))
        P_axis_norm = np.linalg.norm(P_axis)
        P_axis = P_axis / P_axis_norm
        np.nan_to_num(P_axis, copy=False)
    else:
        P_axis = np.array(([0.,0.,0.]))

    if R_axis != [0.0,0.0,0.0]:
        R_axis = np.array(R_axis,dtype=np.dtype('d'))
        R_axis_norm = np.linalg.norm(R_axis)
        R_axis = R_axis / R_axis_norm
        np.nan_to_num(R_axis, copy=False)
    else:
        R_axis = np.array(([0.,0.,0.]))

    # print_div(str(P_axis)+", "+ str(R_axis) + "<br>")

    global is_jogging
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_jog_cartesian_gamepad(P_axis, R_axis))
    # else:
    #     print_div("Jogging has not finished yet..<br>")

async def async_jog_cartesian_gamepad(P_axis, R_axis):
    global plugin_jogCartesianSpace
    await plugin_jogCartesianSpace.async_prepare_jog(None)
        
    global is_gamepadaxisactive
    global is_gamepadbuttondown
    # print_div("here 0<br>")
    while (is_gamepadaxisactive or is_gamepadbuttondown): 
        # Call Jog Cartesian Space Service funtion to handle this jogging
        # await plugin_jogCartesianSpace.async_jog_cartesian(P_axis, R_axis, None)
        await plugin_jogCartesianSpace.async_jog_cartesian2(P_axis, R_axis, None)

    await plugin_jogCartesianSpace.async_stop_joints(None)
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
        await plugin_jogJointSpace.async_jog_joints2(q_i, sign, None)

    await plugin_jogJointSpace.async_stop_joints(None)
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
    
    global plugin_jogJointSpace
    plugin_jogJointSpace.async_jog_joints_zeros(None)

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
    global is_jogging
    global plugin_jogJointSpace

    joint_angles = np.zeros((7,))
    element_id = "j1_angle_in"
    
    for j in range(1,7+1):
        element_id = "j" + str(j) + "_angle_in"
        text_container_angle = document.getElementById(element_id)
        angle = text_container_angle.value # str and in degrees
        try: # if not angle == None or not angle == "":
            joint_angles[j-1] = float(angle)* np.deg2rad(1)
        except: # else:
            print_div("Please specify angle of each joint!<br>")
            is_jogging = False
            return

    await plugin_jogJointSpace.async_jog_joints_to_angles(joint_angles,None)

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
    await plugin_jogCartesianSpace.async_prepare_jog(None)
    # await plugin_jogCartesianSpace.async_jog_cartesian(P_axis, R_axis, None)
    
    global is_mousedown
    while (is_mousedown):
        # Call Jog Cartesian Space Service funtion to handle this jogging
        # await plugin_jogCartesianSpace.async_jog_cartesian(P_axis, R_axis, None)
        await plugin_jogCartesianSpace.async_jog_cartesian2(P_axis, R_axis, None)

    await plugin_jogCartesianSpace.async_stop_joints(None)
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
# ---------------------------END: CARTESIAN SPACE JOGGING --------------------------- #

# ---------------------------BEGIN: SAVE PLAYBACK POSES --------------------------- #
def save_cur_pose_func(self):
    print_div('Saving to "Saved Poses" list..<br>')
    loop.call_soon(async_save_cur_pose_func())
    
async def async_save_cur_pose_func():
    # Get the current joint angles as ndarray and str
    # _, joints_text = await update_joint_info() # Current Joint angles in radian ndarray, N x 1 and str
    joints_text = await update_joint_info() # Current Joint angles in radian ndarray, N x 1 and str
    joints_text = joints_text[:-1] # Delete the last comma    

    # Add the current joint angles to the saved poses list on web browser UI
    element_id = "saved_poses_list"
    poses_list = document.getElementById(element_id)
    option = document.createElement("option")
    option.text = joints_text
    poses_list.add(option)

    # Save the cur pose to plug in as well
    global plugin_savePlayback
    await plugin_savePlayback.async_save_cur_pose(None)

def go_sel_pose_func(self):
    print_div("Moving to selected pose..<br>")
    global is_jogging
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_go_sel_pose_func())
    else:
        print_div("Jogging has not finished yet..<br>")

async def async_go_sel_pose_func():
    # Read the selected pose index from the browser
    element_id = "saved_poses_list"
    poses_list = document.getElementById(element_id)
    index = poses_list.selectedIndex
    try:
        if index == -1:
            print_div("Please select a pose from Saved Poses.<br>")
        else:
            global plugin_savePlayback
            await plugin_savePlayback.async_go_sel_pose(index,None)
    except:
        pass

    global is_jogging
    is_jogging = False

def playback_poses_func(self):
    print_div("Playing Back Poses..<br>")
    global is_jogging
    if (not is_jogging): 
        is_jogging = True
        loop.call_soon(async_playback_poses_func())
    else:
        print_div("Jogging has not finished yet..<br>")

async def async_playback_poses_func():
    # Get elements, poses and paramaters from web interface
    poses_list = document.getElementById("saved_poses_list")
    num_loops_elem = document.getElementById("num_loops_in")
    num_loops = int(num_loops_elem.value)
    joint_vel_range = document.getElementById("joint_vel_range")
    joint_vel_ratio = float(joint_vel_range.value)/100.0

    # Time to complete the playback
    time_loops_elem = document.getElementById("time_loops_in")
    t_complete = float(time_loops_elem.value) #seconds

    if poses_list.length >= 4:
        global plugin_savePlayback
        await plugin_savePlayback.async_playback_poses(num_loops, joint_vel_ratio, t_complete, None)
    else:
        print_div("You need at least 4 different points. Add some poses to Saved Poses and try again<br>")

    global is_jogging
    is_jogging = False

def del_sel_pose_func(self):
    print_div("Deleting seleted Pose..<br>")
    loop.call_soon(async_del_sel_pose_func())

async def async_del_sel_pose_func():
    # Read the selected pose index from the browser
    element_id = "saved_poses_list"
    poses_list = document.getElementById(element_id)
    index = poses_list.selectedIndex
    try:
        if index == -1:
            print_div("Please select a pose from Saved Poses.<br>")
        else:
            global plugin_savePlayback
            await plugin_savePlayback.async_del_sel_pose(index,None)
            # Delete from UI too.
            poses_list.remove(index); 
    except:
        pass
        
def up_sel_pose_func(self):
    print_div("Move up seleted Pose..<br>")
    loop.call_soon(async_up_sel_pose_func())

async def async_up_sel_pose_func():
    # Read the selected pose index from the browser
    element_id = "saved_poses_list"
    poses_list = document.getElementById(element_id)
    index = poses_list.selectedIndex
    try:
        if index == -1:
            print_div("Please select a pose from Saved Poses.<br>")
        else:
            global plugin_savePlayback
            await plugin_savePlayback.async_up_sel_pose(index,None)
            # Up it from UI too.
            if index > 0:
                option = poses_list.options[index];
                poses_list.remove(index);
                poses_list.add(option,index-1)

    except:
        pass

def down_sel_pose_func(self):
    print_div("Move down seleted Pose..<br>")
    loop.call_soon(async_down_sel_pose_func())

async def async_down_sel_pose_func():
    # Read the selected pose index from the browser
    element_id = "saved_poses_list"
    poses_list = document.getElementById(element_id)
    index = poses_list.selectedIndex
    try:
        if index == -1:
            print_div("Please select a pose from Saved Poses.<br>")
        else:
            global plugin_savePlayback
            await plugin_savePlayback.async_down_sel_pose(index,None)
            # Down it from UI too.
            if index < poses_list.length-1:
                option = poses_list.options[index];
                poses_list.remove(index);
                poses_list.add(option,index+1)

    except:
        pass
# ---------------------------END: SAVE PLAYBACK POSES --------------------------- #

# ---------------------------START: Select Robot --------------------------- #
async def async_select_available_robot_url(robot_urls):
    print_div("Selecting the robot URL.. <br>")
    # Read the selected robot index from the browser  
    element_id = "available_robots"
    available_robots_list = document.getElementById(element_id)
    index = available_robots_list.selectedIndex
    return robot_urls[index]
# ---------------------------END: Select Robot --------------------------- #

async def update_state_flags():
    # For reading robot state flags
    #print_div_flag_info("State Flags Updating..")
    global plugin_updateInfo
    flags_text = await plugin_updateInfo.async_state_flags_str(None)
    print_div_flag_info(flags_text)
  
  
async def update_joint_info():
    # For reading Joint Angles
    # print_div_j_info("Joint info Updating..")
    # global d
    # d_state = await d.robot_state.AsyncPeekInValue(None,5)    
    # d_q = d_state[0].joint_position
    
    global plugin_updateInfo
    joints_text = await plugin_updateInfo.async_current_joint_angles_str(None)
      
    # return d_q, joints_text  # returns d_q in radian ndarray, joints_text str to print to browser.
    return joints_text  # joints_text str to print to browser.

async def update_num_info():
    # For reading about number of robot joints, joint types, joint limits etc
    global plugin_updateInfo
    joint_limits_text = await plugin_updateInfo.async_joint_limits_str_array(None)
    print_div_j_limit_info(joint_limits_text[0], joint_limits_text[1])

    joint_num_type_vel_acc_name_text = await plugin_updateInfo.async_joint_num_type_vel_acc_name_str(None)
    print_div_num_info(joint_num_type_vel_acc_name_text) 
     
async def update_kin_info():        
    global plugin_updateInfo
    HnP_text = await plugin_updateInfo.async_kinematics_str(None)
    print_div_kin_info(HnP_text)

async def update_end_info():
    global plugin_updateInfo
    pose_str = await plugin_updateInfo.async_current_pose_str(None)
    print_div_end_info(pose_str) 
    print_div_cur_pose(pose_str)

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

def stop_robot_func(self):
    global is_stop_robot
    is_stop_robot = True
    print_div("Robot stop is called <br>")

async def client_drive():
    # rr+ws : WebSocket connection without encryption
    # ip = '192.168.50.152' # robot service ip
    # ip = 'localhost' # robot service ip
    # ip = 'localhost'
    
    # ip_plugins = '192.168.50.152' # plugins ip
    ip_plugins = 'localhost' # plugins ip
    # ip_plugins = '128.113.224.154' # plugins ip lab
    # ip_plugins = 'localhost' # plugins ip

    # url ='rr+ws://'+ ip +':58653?service=robot'   # Sawyer simulation
    # url ='rr+ws://128.113.224.23:58654?service=robot' # sawyer in lab

    # url ='rr+ws://'+ ip +':58655?service=robot' #ABB

    # url = 'rr+ws://'+ ip +':23333?service=robot' # Dr.Wasons's Robot (rp260)

    # url = 'rr+ws://[fe80::7c64:bf9f:7c1d:5a9e]:58653/?nodeid=eb42bd99-6352-4784-9769-6a6ea260f558&service=robot'
    # url = 'rr+ws:///?nodeid=b257c6ac-d0f0-444a-9971-e29718605924&service=robot'


    # #_________________________ multiple robot urls _________________
    # ip = '192.168.50.40' # robot service ip
    # # ip = '128.113.224.64' # robot service ip in Lab
    # url_sawyer = 'rr+ws://'+ ip +':58653?service=robot' # Sawyer simulation
    # # url_sawyer = 'rr+ws://'+ ip +':58654?service=robot' # Sawyer simulation in Lab

    # ip = 'localhost' # robot service ip
    # url_rp260 = 'rr+ws://'+ ip +':23333?service=robot'  # Dr.Wasons's Robot (rp260)

    # ip = 'localhost' # robot service ip
    # # ip = '128.113.224.12' # robot service ip in Lab
    # url_abb = 'rr+ws://'+ ip +':58655?service=robot'  # ABB
    # # url_abb = 'rr+ws://'+ ip +':58651?service=robot'  # ABB in Lab

    # ip = 'localhost' # robot service ip
    # # ip = '128.113.224.83' # robot service ip in Lab
    # url_ur5 = 'rr+ws://'+ ip +':58653?service=robot'  # UR5
    # # url_ur5 = 'rr+ws://'+ ip +':58653?service=robot'  # UR5 in Lab

    # # robot_urls = [url_sawyer,url_rp260, url_abb]
    # robot_urls = [url_sawyer,url_ur5, url_abb]

    # url = await async_select_available_robot_url(robot_urls)
    # # print_div('Selected Robot url: '+ url + '<br>')

    # print_div('Program started, please wait..<br>')
    # #_________________________ multiple robot urls _________________

    

    try:
        # Discover Available Robots
        ## Discovery plugin
        print_div('Discovery plugin is connecting (robot)..<br>')

        url_plugin_discovery = 'rr+ws://' + ip_plugins + ':8896?service=Discovery'
        global plugin_discovery
        plugin_discovery = await RRN.AsyncConnectService(url_plugin_discovery,None,None,None,None)
        print_div('discovery plugin is connected (robot)..<br>')

        RobotConnectionURLs = await plugin_discovery.async_available_robot_ConnectionURLs(None)
        # print_div(str(RobotConnectionURLs))
        # RobotNames = await plugin_discovery.async_available_robot_Names(None)
        # print_div("Available Robots:<br>"+str(RobotNames)+ "<br>" )
        # RobotNodeNames = await plugin_discovery.async_available_robot_NodeNames(None)
        # print_div("Available Robots:<br>"+str(RobotNodeNames)+ "<br>" )

        # Trying -----
        url = await async_select_available_robot_url(RobotConnectionURLs)
        print_div('Selected Robot url: '+ url + '<br>')
        # ------------

        ToolConnectionURLs = await plugin_discovery.async_available_tool_ConnectionURLs(None)
        tool_node_names_lst = await plugin_discovery.async_available_tool_NodeNames(None)

        print_div("ToolConnectionURLs" + str(ToolConnectionURLs) + "<br>")
        print_div("tool_node_names_lst" + str(tool_node_names_lst) + "<br>")

       
        #Connect to the service
        # global d # d is the robot object from RR
        # d = await RRN.AsyncConnectService(url,None,None,None,None)
        # d.async_reset_errors(None)
        # d.async_enable(None)
        
        # # Define Robot modes
        # global robot_const, halt_mode, jog_mode, position_mode, trajectory_mode
        # robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", d)
        # halt_mode = robot_const["RobotCommandMode"]["halt"]
        # jog_mode = robot_const["RobotCommandMode"]["jog"]
        # position_mode = robot_const["RobotCommandMode"]["velocity_command"]
        # trajectory_mode = robot_const["RobotCommandMode"]["trajectory"]

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

        ## UpdateInfo plugin
        print_div('UpdateInfo plugin is connecting..<br>')

        # url_plugin_updateInfo = 'rr+ws://localhost:8895?service=UpdateInfo'
        url_plugin_updateInfo = 'rr+ws://' + ip_plugins + ':8895?service=UpdateInfo'
        global plugin_updateInfo
        plugin_updateInfo = await RRN.AsyncConnectService(url_plugin_updateInfo,None,None,None,None)
        await plugin_updateInfo.async_connect2robot(url,None)
        print_div('UpdateInfo plugin is connected..<br>')

        ## JogJointSpace plugin
        print_div('JogJointSpace plugin is connecting..<br>')

        # url_plugin_jogJointSpace = 'rr+ws://localhost:8890?service=JogJointSpace'
        url_plugin_jogJointSpace = 'rr+ws://' + ip_plugins + ':8890?service=JogJointSpace'
        global plugin_jogJointSpace
        plugin_jogJointSpace = await RRN.AsyncConnectService(url_plugin_jogJointSpace,None,None,None,None)
        await plugin_jogJointSpace.async_connect2robot(url,None)
        print_div('JogJointSpace plugin is connected..<br>')

        ## JogCartesianSpace plugin
        print_div('JogCartesianSpace plugin is connecting..<br>')

        # url_plugin_jogCartesianSpace = 'rr+ws://localhost:8891?service=JogCartesianSpace'
        url_plugin_jogCartesianSpace = 'rr+ws://' + ip_plugins + ':8891?service=JogCartesianSpace'
        global plugin_jogCartesianSpace
        plugin_jogCartesianSpace = await RRN.AsyncConnectService(url_plugin_jogCartesianSpace,None,None,None,None)
        await plugin_jogCartesianSpace.async_connect2robot(url,None)
        print_div('JogJointSpace plugin is connected..<br>')

        ## SavePlayback plugin
        print_div('SavePlayback plugin is connecting..<br>')

        # url_plugin_savePlayback = 'rr+ws://localhost:8894?service=SavePlayback'
        url_plugin_savePlayback = 'rr+ws://' + ip_plugins + ':8894?service=SavePlayback'
        global plugin_savePlayback
        plugin_savePlayback = await RRN.AsyncConnectService(url_plugin_savePlayback,None,None,None,None)
        await plugin_savePlayback.async_connect2robot(url,None)
        print_div('SavePlayback plugin is connected..<br>')


        ## Tool plugin
        print_div('Tool plugin is connecting..<br>')

        url_plugin_tool = 'rr+ws://' + ip_plugins + ':8900?service=Tool'
        # url_plugin_tool = ''
        if not url_plugin_tool == '':
            global plugin_tool
            plugin_tool = await RRN.AsyncConnectService(url_plugin_tool,None,None,None,None)
            await plugin_tool.async_connect2robot(url,None)
            await plugin_tool.async_connect2all_tools(ToolConnectionURLs,tool_node_names_lst,None)
            active_tool_url_debug = await plugin_tool.async_connect2active_tool(None)
            
            print_div("active_tool_url_debug: " + str(active_tool_url_debug) + "<br>")
            print_div('Tool plugin is connected..<br>')

        
        ## Simulation Tool Link Attacher plugin
        print_div('Simulation Tool Link Attacher plugin is connecting..<br>')

        # url_plugin_toolLinkAttacher = 'rr+ws://' + ip_plugins + ':8899?service=ToolLinkAttacher'
        url_plugin_toolLinkAttacher = ''
        if not url_plugin_toolLinkAttacher == '':
            global plugin_toolLinkAttacher
            plugin_toolLinkAttacher = await RRN.AsyncConnectService(url_plugin_toolLinkAttacher,None,None,None,None)
            print_div('ToolLinkAttacher plugin is connected..<br>')

        


        ## Blockly plugin
        print_div('Blockly plugin is connecting..<br>')

        url_plugin_blockly = 'rr+ws://' + ip_plugins + ':8897?service=Blockly'
        global plugin_blockly
        plugin_blockly = await RRN.AsyncConnectService(url_plugin_blockly,None,None,None,None)
        
        url_plugins_lst = [url_plugin_jogJointSpace, url_plugin_jogCartesianSpace, url_plugin_savePlayback, url_plugin_tool, url_plugin_toolLinkAttacher]
        await plugin_blockly.async_connect2plugins(url_plugins_lst,None)
        # Update blockly saved workspaces both on UI and RunTime
        await async_update_blockly_saved_workspaces() 
        print_div('Blockly plugin is connected..<br>')

        

        # PLUGIN SERVICE CONNECTIONS END__________________________________
         
        print_div('READY!<br>')
        
        # Get the current joint positions
        # _, joints_text = await update_joint_info() # Joint angles in radian ndarray, N x 1 and str
        joints_text = await update_joint_info() # Joint angles in radian ndarray, N x 1 and str
        print_div_j_info(joints_text)
                  
        # global num_joints, joint_types, joint_lower_limits, joint_upper_limits, joint_vel_limits, joint_acc_limits, joint_names
        # # Get the number of Joints, Joint Types, Limits etc in the robot.
        # num_joints, joint_types, joint_lower_limits, joint_upper_limits, joint_vel_limits, joint_acc_limits, joint_names  = await update_num_info()
        await update_num_info()
        
        # Get the kinematics info, P and H in product of exponentials convention
        await update_kin_info()
        
        # UPdate the end effector pose info
        await update_end_info()
            
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
        
        # Playback Poses Buttons
        button_save_cur_pose = document.getElementById("save_pose_btn")
        button_go_sel_pose = document.getElementById("go_sel_pose_btn")
        button_playback_poses = document.getElementById("playback_poses_btn")
        button_del_sel_pose = document.getElementById("del_sel_pose_btn")
        button_up_sel_pose = document.getElementById("up_sel_pose_btn")
        button_down_sel_pose = document.getElementById("down_sel_pose_btn")

        # Blockly Execute buttons
        button_execute_blockly = document.getElementById("execute_blockly_btn") 
        button_load_blocks = document.getElementById("load_blocks_btn") 
        button_delete_blocks = document.getElementById("delete_blocks_btn")
        button_edit_blocks_name = document.getElementById("edit_blocks_name_btn")
        button_save_blocks = document.getElementById("save_blocks_btn")
        button_save_as_blocks = document.getElementById("save_as_blocks_btn")
        button_clear_blocks = document.getElementById("clear_blocks_btn")

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
        
        # Playback Poses Buttons event listeners
        button_save_cur_pose.addEventListener("click", save_cur_pose_func)
        button_go_sel_pose.addEventListener("click", go_sel_pose_func)
        button_playback_poses.addEventListener("click", playback_poses_func)
        button_del_sel_pose.addEventListener("click", del_sel_pose_func)
        button_up_sel_pose.addEventListener("click", up_sel_pose_func)
        button_down_sel_pose.addEventListener("click", down_sel_pose_func)

        button_execute_blockly.addEventListener("click", execute_blockly_func)
        button_load_blocks.addEventListener("click", load_blocks_func)
        button_delete_blocks.addEventListener("click", delete_blocks_func)
        button_edit_blocks_name.addEventListener("click", edit_blocks_name_func)
        button_save_blocks.addEventListener("click", save_blocks_func)
        button_save_as_blocks.addEventListener("click", save_as_blocks_func)
        button_clear_blocks.addEventListener("click", clear_blocks_func)

        global is_jogging
        is_jogging = False
        global is_mousedown
        is_mousedown = False        

        document.addEventListener("mouseup", mouseup_func)

        global is_gamepadbuttondown
        is_gamepadbuttondown = False
        global is_gamepadaxisactive
        is_gamepadaxisactive = False

        # ---------------------------
        # Element reference for start robot button
        button_start_robot = document.getElementById("start_robot_btn")
        # Event listener for stop robot func
        button_start_robot.addEventListener("mousedown", stop_robot_func)

        global is_stop_robot 
        is_stop_robot = False
        # ---------------------------


        while not is_stop_robot:
            # Update joint angles
            # _, joints_text = await update_joint_info() # Joint angles in radian ndarray, N x 1 and str
            joints_text = await update_joint_info() # Joint angles as str
            print_div_j_info(joints_text)
            
            # UPdate the end effector pose info
            await update_end_info()
            
            await update_state_flags()

            


        # Remove all event listeners before return
        button_stop.removeEventListener("click", stop_func)
        button_j1_pos.removeEventListener("mousedown", j1_pos_func)
        button_j1_neg.removeEventListener("mousedown", j1_neg_func)
        button_j2_pos.removeEventListener("mousedown", j2_pos_func)
        button_j2_neg.removeEventListener("mousedown", j2_neg_func)
        button_j3_pos.removeEventListener("mousedown", j3_pos_func)
        button_j3_neg.removeEventListener("mousedown", j3_neg_func)
        button_j4_pos.removeEventListener("mousedown", j4_pos_func)
        button_j4_neg.removeEventListener("mousedown", j4_neg_func)
        button_j5_pos.removeEventListener("mousedown", j5_pos_func)
        button_j5_neg.removeEventListener("mousedown", j5_neg_func)
        button_j6_pos.removeEventListener("mousedown", j6_pos_func)
        button_j6_neg.removeEventListener("mousedown", j6_neg_func)
        button_j7_pos.removeEventListener("mousedown", j7_pos_func) 
        button_j7_neg.removeEventListener("mousedown", j7_neg_func)
        button_angles_submit.removeEventListener("click", move_to_angles_func)
        button_X_pos.removeEventListener("mousedown", X_pos_func)
        button_X_neg.removeEventListener("mousedown", X_neg_func)
        button_Y_pos.removeEventListener("mousedown", Y_pos_func)
        button_Y_neg.removeEventListener("mousedown", Y_neg_func)
        button_Z_pos.removeEventListener("mousedown", Z_pos_func)
        button_Z_neg.removeEventListener("mousedown", Z_neg_func)
        button_theta_X_pos.removeEventListener("mousedown", tX_pos_func)
        button_theta_X_neg.removeEventListener("mousedown", tX_neg_func)
        button_theta_Y_pos.removeEventListener("mousedown", tY_pos_func)
        button_theta_Y_neg.removeEventListener("mousedown", tY_neg_func)
        button_theta_Z_pos.removeEventListener("mousedown", tZ_pos_func)
        button_theta_Z_neg.removeEventListener("mousedown", tZ_neg_func)
        button_save_cur_pose.removeEventListener("click", save_cur_pose_func)
        button_go_sel_pose.removeEventListener("click", go_sel_pose_func)
        button_playback_poses.removeEventListener("click", playback_poses_func)
        document.removeEventListener("mouseup", mouseup_func)
        button_start_robot.removeEventListener("mousedown", stop_robot_func)
        button_del_sel_pose.removeEventListener("click", del_sel_pose_func)
        button_up_sel_pose.removeEventListener("click", up_sel_pose_func)
        button_down_sel_pose.removeEventListener("click", down_sel_pose_func)
        button_execute_blockly.removeEventListener("click", execute_blockly_func)
        button_load_blocks.removeEventListener("click", load_blocks_func)
        button_delete_blocks.removeEventListener("click", delete_blocks_func)
        button_edit_blocks_name.removeEventListener("click", edit_blocks_name_func)
        button_save_blocks.removeEventListener("click", save_blocks_func)
        button_save_as_blocks.removeEventListener("click", save_as_blocks_func)
        button_clear_blocks.removeEventListener("click", clear_blocks_func)

        return
            
    except:
        import traceback
        print_div(traceback.format_exc())
        raise

loop = RR.WebLoop()
loop.call_soon(client_drive())

# RR.WebLoop.run(client_drive())
