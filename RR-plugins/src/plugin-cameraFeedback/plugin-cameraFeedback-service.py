import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

class CameraFeedback_impl(object):
    def __init__(self):
        self.null = None 
        # TODO:
        # print("HELLOO")

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-cameraFeedback-service", 8889) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginCameraFeedback")

        # create object
        CameraFeedback_inst = CameraFeedback_impl()
        # register service with service name "CameraFeedback", type "experimental.pluginCameraFeedback.CameraFeedback", actual object: CameraFeedback_inst
        RRN.RegisterService("CameraFeedback","experimental.pluginCameraFeedback.CameraFeedback",CameraFeedback_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginCameraFeedback Server started, press enter to quit...")
        else:
            raw_input("pluginCameraFeedback Server started, press enter to quit...")

if __name__ == '__main__':
    main()