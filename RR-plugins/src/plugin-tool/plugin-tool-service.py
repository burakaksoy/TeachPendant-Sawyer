import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np
import time

import RobotRaconteurCompanion as RRC
from RobotRaconteurCompanion.Util.IdentifierUtil import IdentifierUtil



class Tool_impl(object):
    def __init__(self):
        self.url_robot = None
        self.robot = None ## RR robot object
        self.is_robot_connected = False

        self.active_tool_identifier = None

        self.tool_name_url_dict = {} # Dictionary for connected tool urls(key:node names)
        self.tool_objs_dict = {} # Dictionary for connected tool objects(key:node names)
        self.tool_identifiers_dict = {} # Dictionary for connected tool objects device identifiers(key:node names)
        self.is_tools_connected = False
        
        self.active_tool = None # Active tool object instance
        self.active_tool_url = None
        self.is_active_tool_connected = False


        self._identifier_util = IdentifierUtil(RRN)

    def reset(self):
        self.url_robot = None
        self.robot = None ## RR robot object
        self.is_robot_connected = False
    
        self.active_tool_identifier = None

    def reset_connected_tools(self):
        self.tool_name_url_dict = {} # Dictionary for connected tool urls(key:node names)
        self.tool_objs_dict = {} # Dictionary for connected tool objects(key:node names)
        self.tool_identifiers_dict = {} # Dictionary for connected tool objects device identifiers(key:node names)
        self.is_tools_connected = False
        
        self.active_tool = None # Active tool object instance
        self.active_tool_url = None
        self.is_active_tool_connected = False

    def connect2robot(self, url_robot):
        if self.robot is None:
            self.url_robot = url_robot

            # self.robot = RRN.ConnectService(self.url_robot) # connect to robot with the given url
            self.robot_sub = RRN.SubscribeService(self.url_robot)
            self.robot = self.robot_sub.GetDefaultClientWait(1) 
            
            self.is_robot_connected = True

            # Find the active tool identifier of the robot
            try:
                self.active_tool_identifier =  self.robot.robot_info.chains[0].current_tool.device_info.device
            except:
                import traceback
                print(traceback.format_exc())
                print("active tool identifier could not found from the robot, check whether robot is connected to a tool")
            # log that the robot is successfully connected  
            print("Robot is connected to plugin tool service!")
        else:
            # Give an error that says the robot is already connected
            print("Robot is already connected to plugin tool service! Trying to connect again..")
            self.reset()
            self.connect2robot(url_robot)

    def connect2all_tools(self, tool_connection_urls_lst, tool_node_names_lst):
        if not self.is_tools_connected: # if the dictionary is empty
            self.tool_name_url_dict = dict(zip(tool_node_names_lst,tool_connection_urls_lst)) 

            for tool_name, tool_url in self.tool_name_url_dict.items():
                # connect to the tool service url
                tool_obj = RRN.ConnectService(tool_url) # connect to tool with given url
                # add the connected tool object to tool object dictionary
                self.tool_objs_dict[tool_name] = tool_obj
                # also create the self.tool_identifiers_dict
                self.tool_identifiers_dict[tool_name] = tool_obj.device_info.device

            self.is_tools_connected = True
            # log that the tools are successfully connected
            print("All tools are connected to plugin-tool-service!")

        else:
            # Give an error that says the tools are already connected
            print("tools are already connected to plugin-tool-service! Trying to connect again..")
            self.reset_connected_tools()
            self.connect2all_tools(tool_connection_urls_lst, tool_node_names_lst)

    def connect2active_tool(self):
        if self.is_tools_connected and self.is_robot_connected:
            # match the identifiers between the all tools and the active robot tool to register the active tool
            for tool_name, tool_identifier in self.tool_identifiers_dict.items():
                is_match = self._identifier_util.IsIdentifierMatch(tool_identifier, self.active_tool_identifier)
                print("is_match: " + str(is_match))

                # Note: IsIdentifierMatch is designed to consider wildcards
                # so if one identifier has an empty name or all zero UUID, it will match any value
                # if both are filled in, it will give a direct true or false

                if is_match:
                    self.active_tool = self.tool_objs_dict[tool_name]
                    self.active_tool_url = self.tool_name_url_dict[tool_name]
                    self.is_active_tool_connected = True
                    return self.active_tool_url
        else:
            # Give an error message to show that the robot or the tools are not connected
            print("Robot or the tools are not connected to plugin tool service yet!")
            return "" # return empty string for the connection url of the tool for the match


    def active_tool_open(self):
        if self.is_active_tool_connected:
            self.active_tool.open()
        else:
            print("Active tool is not connected to tool plugin yet, make sure you executed connect2robot(...),connect2all_tools(...), connect2active_tool() functions first ")

    def active_tool_close(self):
        if self.is_active_tool_connected:
            self.active_tool.close()
        else:
            print("Active tool is not connected to tool plugin yet, make sure you executed connect2robot(...),connect2all_tools(...), connect2active_tool() functions first ")        

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-tool-service", 8900) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginTool")
        RRC.RegisterStdRobDefServiceTypes(RRN)

        # create object
        Tool_inst = Tool_impl()
        # register service with service name "Tool", type "experimental.pluginTool.Tool", actual object: Tool_inst
        RRN.RegisterService("Tool","experimental.pluginTool.Tool",Tool_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginTool Server started, press enter to quit...")
        else:
            raw_input("pluginTool Server started, press enter to quit...")

if __name__ == '__main__':
    main()