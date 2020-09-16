import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect to robot 
import numpy as np

class JogCartesianSpace_impl(object):
    def __init__(self):
        self.null = None 
        # TODO:

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