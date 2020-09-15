import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect to robot 
import numpy as np

class Vision_impl(object):
    def __init__(self):
        self.null = None 
        # TODO:

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-vision-RR-v0.9-service", 8889) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginVision")

        # create object
        Vision_inst = Vision_impl()
        # register service with service name "Vision", type "experimental.pluginVision.Vision", actual object: Vision_inst
        RRN.RegisterService("Vision","experimental.pluginVision.Vision",Vision_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("Server started, press enter to quit...")
        else:
            raw_input("Server started, press enter to quit...")

if __name__ == '__main__':
    main()