import sys
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
from RobotRaconteur.Client import *     #import RR client library to connect 
import numpy as np

class CameraTracking_impl(object):
    def __init__(self):
        self.null = None 
        # TODO:

def main():
    # RR.ServerNodeSetup("NodeName", TCP listen port, optional set of flags as parameters)
    with RR.ServerNodeSetup("experimental.plugin-cameraTracking-service", 8898) as node_setup:

        # register service type
        RRN.RegisterServiceTypeFromFile("./experimental.pluginCameraTracking")

        # create object
        CameraTracking_inst = CameraTracking_impl()
        # register service with service name "CameraTracking", type "experimental.pluginCameraTracking.CameraTracking", actual object: CameraTracking_inst
        RRN.RegisterService("CameraTracking","experimental.pluginCameraTracking.CameraTracking",CameraTracking_inst)

        #Wait for the user to shutdown the service
        if (sys.version_info > (3, 0)):
            input("pluginCameraTracking Server started, press enter to quit...")
        else:
            raw_input("pluginCameraTracking Server started, press enter to quit...")

if __name__ == '__main__':
    main()