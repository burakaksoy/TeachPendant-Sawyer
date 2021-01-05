# Client for Robot
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
# import time
import numpy as np
import sys
# import math

# sys.path.append("./my_source.zip")
# import general_robotics_toolbox as rox


class ClientRobot(object):
    """Client Class to access client data in a more convenient way"""
    def __init__(self, ip_robot, ip_plugins, port_robot):
        # Service IPs
        self.ip_cam = ip_cam
        self.ip_plugins = ip_plugins

        # Port numbers for robot types
        self.port_robot_service = '2355'
        self.port_pluginCameraFeedback_service = '8889'
        self.port_pluginCameraTraining_service = '8892'
        self.port_pluginCameraCalibration_service = '8893'



async def client_robot():
    # ip_robot = 'localhost'
    # ip_robot = '192.168.50.152'
    # ip_robot = '192.168.50.40'
    ip_robot = 'localhost'
    
    # ip_plugins = 'localhost'
    # ip_plugins = '192.168.50.152'
    ip_plugins = 'localhost'
    
    try:
        # Run the client as a class to access client data in a more convenient way
        cli_robot = ClientRobot(ip_robot,ip_plugins) 
        
    except:
        import traceback
        print_div(traceback.format_exc())
        raise

loop = RR.WebLoop()
loop.call_soon(client_robot())
# RR.WebLoop.run(client_robot())

# RRN.PostToThreadPool(client_robot()) 