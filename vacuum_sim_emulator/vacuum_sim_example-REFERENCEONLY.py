from RobotRaconteur.Client import *

import numpy as np
from importlib import import_module
import time, traceback, sys, yaml, argparse, copy
vacuum_inst=RRN.ConnectService('rr+tcp://localhost:50000/?service=vacuumlink')
robot_name = 'sawyer'

obj_name = 'toothpaste'
vacuum_inst.vacuum(robot_name,obj_name,1)
vacuum_inst.vacuum(robot_name,obj_name,0)

obj_name = 'perfume'
vacuum_inst.vacuum(robot_name,obj_name,1)
vacuum_inst.vacuum(robot_name,obj_name,0)

obj_name = 'round_bottle'
vacuum_inst.vacuum(robot_name,obj_name,1)
vacuum_inst.vacuum(robot_name,obj_name,0)

obj_name = 'soap_bar'
vacuum_inst.vacuum(robot_name,obj_name,1)
vacuum_inst.vacuum(robot_name,obj_name,0)
