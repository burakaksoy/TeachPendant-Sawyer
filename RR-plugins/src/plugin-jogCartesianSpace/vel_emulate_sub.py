from RobotRaconteur.Client import *
import time
import numpy as np
import threading
import traceback

class EmulatedVelocityControl(object):
    def __init__(self, robot, dt = 0.01):
        self._seqno = 0
        # self._state_wire 

        # self._pos_command_wire = pos_command_wire
        self._this_lock = threading.Lock()
        self._vel_mode_enabled=False
        self._vel_mode_pos = None
        self._dt = dt
        self._vel_mode_command = None
        self.RobotJointCommand = RRN.GetStructureType("com.robotraconteur.robotics.robot.RobotJointCommand",robot)
        self.robot = robot


    def joint_position(self):
        cur_robot_state = self.robot.robot_state.PeekInValue()
        cur_q = cur_robot_state[0].joint_position
        return cur_q # in radian ndarray
        # return self._state_wire.InValue.joint_position
    def robot_pose(self):
        cur_robot_state = self.robot.robot_state.PeekInValue()
        return cur_robot_state[0].kin_chain_tcp[0]
        # return self._state_wire.InValue.kin_chain_tcp[0]

    def set_joint_command_position(self, q):
        with self._this_lock:
            joint_cmd1 = self.RobotJointCommand()
            self._seqno += 1
            joint_cmd1.seqno = self._seqno

            # joint_cmd1.state_seqno = self._state_wire.InValue.seqno
            cur_robot_state = self.robot.robot_state.PeekInValue()  
            joint_cmd1.state_seqno = cur_robot_state[0].seqno

            joint_cmd1.command = q

            self.robot.position_command.PokeOutValue(joint_cmd1)
            # self._pos_command_wire.SetOutValueAll(joint_cmd1) 

    def enable_velocity_mode(self):
        with self._this_lock:
            self._vel_mode_enabled = True
            self._vel_mode_thread = threading.Thread(target=self._vel_thread)
            # self._vel_mode_pos = self._state_wire.InValue.joint_position
            self._vel_mode_pos = self.joint_position()

            self._vel_mode_thread.daemon=True
            self._vel_mode_thread.start()

    def disable_velocity_mode(self):
        with self._this_lock:
            self._vel_mode_enabled = False
        self._vel_mode_thread.join()

    def set_velocity_command(self, q_dot):
        self._vel_mode_command = q_dot

    def _vel_thread(self):
        try:
            while self._vel_mode_enabled:
                if self._vel_mode_command is not None:
                    self._vel_mode_pos = self._vel_mode_pos + self._vel_mode_command * self._dt
                    self.set_joint_command_position(self._vel_mode_pos)
                time.sleep(self._dt)
        except:
            traceback.print_exc()
            raise
