import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
# import numpy as np
import time

class Discovery_impl(object):
    def __init__(self):
        self.transportschemes = ["rr+local","rr+tcp","rrs+tcp"] 
        # self.transportscheme_desired = "rr+ws"

        # For robots 
        self.service_type = "com.robotraconteur.robotics.robot.Robot" # Discover robots

        self.connectionURLs = [] # Available robot connection URLs
        self.Names = [] # Available robot names 
        self.NodeNames = [] # Available robot nodeNames (sawyer, abb, etc..)

        self.autodiscover() # discover the robots 
        
        # For cameras
        self.service_type_cams = "com.robotraconteur.imaging.Camera" # Discover robots

        self.connectionURLs_cams = [] # Available camera connection URLs
        self.Names_cams = [] # Available camera names 
        self.NodeNames_cams = [] # Available camera nodeNames (camera1, camera2, etc..)

        self.autodiscover_cams() # discover the robots 

    # -----------------------------------------------------------
    # For robots
    def autodiscover(self):
        self.res=RRN.FindServiceByType(self.service_type, self.transportschemes)
        for serviceinfo2 in self.res:
            print(serviceinfo2.NodeID) 
            print(serviceinfo2.NodeName)
            print(serviceinfo2.Name)
            print(serviceinfo2.RootObjectType) 
            print(serviceinfo2.RootObjectImplements)
            print(serviceinfo2.ConnectionURL)
            print(serviceinfo2.ConnectionURL[0])
            print("-------------------------------")

        self.connectionURLs = []
        for serviceinfo2 in self.res:
            # Replace the transportsheme with the desired transport sheme
            url = serviceinfo2.ConnectionURL[0]
            # for tscheme in self.transportschemes:
            #     url = url.replace(tscheme, self.transportscheme_desired, 1)
            
            self.connectionURLs.append(url)

        self.Names = []
        for serviceinfo2 in self.res:
            self.Names.append(serviceinfo2.Name)

        self.NodeNames = []
        for serviceinfo2 in self.res:
            self.NodeNames.append(serviceinfo2.NodeName)

    # function string{list} available_robot_ConnectionURLs()
    def available_robot_ConnectionURLs(self):
        # self.autodiscover()
        return self.connectionURLs
    
    # function string{list} available_robot_Names()
    def available_robot_Names(self):
        # self.autodiscover()        
        return self.Names

    # function string{list} available_robot_NodeNames()
    def available_robot_NodeNames(self):
        # self.autodiscover()
        return self.NodeNames

    # -----------------------------------------------------------
    # For cameras 
    def autodiscover_cams(self):
        self.res_cams = RRN.FindServiceByType(self.service_type_cams, self.transportschemes)
        for serviceinfo2 in self.res_cams:
            print(serviceinfo2.NodeID) 
            print(serviceinfo2.NodeName)
            print(serviceinfo2.Name)
            print(serviceinfo2.RootObjectType) 
            print(serviceinfo2.RootObjectImplements)
            print(serviceinfo2.ConnectionURL)
            print(serviceinfo2.ConnectionURL[0])
            print("-------------------------------")

        self.connectionURLs_cams = []
        for serviceinfo2 in self.res_cams:
            # Replace the transportsheme with the desired transport sheme
            url = serviceinfo2.ConnectionURL[0]
            # for tscheme in self.transportschemes:
            #     url = url.replace(tscheme, self.transportscheme_desired, 1)
            
            self.connectionURLs_cams.append(url)

        self.Names_cams = []
        for serviceinfo2 in self.res_cams:
            self.Names_cams.append(serviceinfo2.Name)

        self.NodeNames_cams = []
        for serviceinfo2 in self.res_cams:
            self.NodeNames_cams.append(serviceinfo2.NodeName)

    # function string{list} available_camera_ConnectionURLs()
    def available_camera_ConnectionURLs(self):
        # self.autodiscover()
        return self.connectionURLs_cams
    
    # function string{list} available_camera_Names()
    def available_camera_Names(self):
        # self.autodiscover()        
        return self.Names_cams

    # function string{list} available_camera_NodeNames()
    def available_camera_NodeNames(self):
        # self.autodiscover()
        return self.NodeNames_cams

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-discovery-service", 8896) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginDiscovery")

        # create object
        Discovery_inst = Discovery_impl()
        # register service with service name "Discovery", type "experimental.pluginDiscovery.Discovery", actual object: Discovery_inst
        RRN.RegisterService("Discovery","experimental.pluginDiscovery.Discovery",Discovery_inst)

        # # These are for using the service on Web Browsers
        # node_setup.tcp_transport.AddWebSocketAllowedOrigin("http://localhost")
        # node_setup.tcp_transport.AddWebSocketAllowedOrigin("http://localhost:8000")
        # node_setup.tcp_transport.AddWebSocketAllowedOrigin("https://johnwason.github.io")

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginDiscovery Server started, press enter to quit...")
        else:
            raw_input("pluginDiscovery Server started, press enter to quit...")

if __name__ == '__main__':
    main()