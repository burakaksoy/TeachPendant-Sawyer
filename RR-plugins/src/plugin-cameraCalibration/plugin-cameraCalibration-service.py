import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

class CameraCalibration_impl(object):
    def __init__(self):
        self.null = None 
        # TODO:

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-cameraCalibration-service", 8893) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginCameraCalibration")

        # create object
        CameraCalibration_inst = CameraCalibration_impl()
        # register service with service name "CameraCalibration", type "experimental.pluginCameraCalibration.CameraCalibration", actual object: CameraCalibration_inst
        RRN.RegisterService("CameraCalibration","experimental.pluginCameraCalibration.CameraCalibration",CameraCalibration_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginCameraCalibration Server started, press enter to quit...")
        else:
            raw_input("pluginCameraCalibration Server started, press enter to quit...")

if __name__ == '__main__':
    main()