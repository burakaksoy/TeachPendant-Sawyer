import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np
import time


class ToolLinkAttacher_impl(object):
    def __init__(self):
        self.tool_name_url_dict = {} # Dictionary for connected tool urls(key:node names)
        self.tool_objs_dict = {} # Dictionary for connected tool objects(key:node names)
        self.is_tools_connected = False

        # Hardcoded url tool for now
        self.url_hardcoded_tool = "rr+tcp://localhost:50000/?service=vacuumlink"
        self.node_name_hardcoded_tool = "vacuumer"
        self.connect2all_tools([self.url_hardcoded_tool], [self.node_name_hardcoded_tool])

    def reset_connected_tools(self):
        self.tool_name_url_dict = {} # Dictionary for connected tool urls(key:node names)
        self.tool_objs_dict = {} # Dictionary for connected tool objects(key:node names)
        self.is_tools_connected = False

    def connect2all_tools(self, tool_connection_urls_lst, tool_node_names_lst):
        if not self.is_tools_connected: # if the dictionary is empty
            self.tool_name_url_dict = dict(zip(tool_node_names_lst,tool_connection_urls_lst)) 

            for tool_name, tool_url in self.tool_name_url_dict.items():
                # connect to the tool service url
                tool_obj = RRN.ConnectService(tool_url) # connect to tool with given url
                # add the connected tool object to tool object dictionary
                self.tool_objs_dict[tool_name] = tool_obj

            self.is_tools_connected = True
            # log that the tools are successfully connected
            print("All tools are connected to ... service!")

        else:
            # Give an error that says the vision plugins are already connected
            print("tools are already connected to ... service! Trying to connect again..")
            self.reset_connected_tools()
            self.connect2all_tools(tool_connection_urls_lst, tool_node_names_lst)

    def attach_link(self, robot_name, obj_name, action):
        print("attach_link with action: " + str(action))

        tool_name = self.node_name_hardcoded_tool
        tool = self.tool_objs_dict[tool_name]# tool object
        tool.vacuum(robot_name, obj_name, action)
        
        print("attach_link with action: " + str(action) + " completed.")

        


def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-toolLinkAttacher-service", 8899) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginToolLinkAttacher")

        # create object
        ToolLinkAttacher_inst = ToolLinkAttacher_impl()
        # register service with service name "ToolLinkAttacher", type "experimental.pluginToolLinkAttacher.ToolLinkAttacher", actual object: ToolLinkAttacher_inst
        RRN.RegisterService("ToolLinkAttacher","experimental.pluginToolLinkAttacher.ToolLinkAttacher",ToolLinkAttacher_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginToolLinkAttacher Server started, press enter to quit...")
        else:
            raw_input("pluginToolLinkAttacher Server started, press enter to quit...")

if __name__ == '__main__':
    main()