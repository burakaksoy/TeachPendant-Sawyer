import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

class CameraTraining_impl(object):
    def __init__(self):
        self.null = None 
        # TODO:

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-cameraTraining-service", 8892) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginCameraTraining")

        # create object
        CameraTraining_inst = CameraTraining_impl()
        # register service with service name "CameraTraining", type "experimental.pluginCameraTraining.CameraTraining", actual object: CameraFeedback_inst
        RRN.RegisterService("CameraTraining","experimental.pluginCameraTraining.CameraTraining",CameraTraining_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginCameraTrainingServer started, press enter to quit...")
        else:
            raw_input("pluginCameraTraining Server started, press enter to quit...")

if __name__ == '__main__':
    main()