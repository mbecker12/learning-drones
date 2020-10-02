"""
Created by Marvin Becker, 15.08.2020
marvinbecker[at]mail.com

    .-.      .-.
   | 0 |    | 1 |
    'T' ____ 'T'
         HH
     _  ____  _
   | 3 |    | 2 |
    ' '      ' '


Python 3.6.5
Library version:
numpy 1.17.4

Implement a omniscient control unit

Taking as inputs sensor data and in later iterations even vision data.
Then use this data to translate it to thruster output

Can be implemented by neural networks and 
trained using neuroevolution or reinforcement learning
"""
import abc
import numpy as np
import math
from drone.sensor.sensor import Sensor, get_positions_and_angles
from drone.quadcopter.geometry import rotation_matrix
from drone.quadcopter.physical_model import QuadcopterPhysics


class ControlUnit(abc.ABC):
    def __init__(self):
        pass

    @abc.abstractmethod
    def translate_input_to_thrust(
        self, sensor_outputs: np.ndarray, rotation: np.ndarray
    ):
        return NotImplementedError


class StraightAhead(ControlUnit):
    def __init__(self, pids: list, quadcopter: QuadcopterPhysics):
        """
        Test module to verify that the physics model works correct.
        """
        # self.qc = quadcopter
        self.time_step = 0
        pass

    def translate_input_to_thrust(
        self,
        diff_coin_drone,
        lab_lin_vel,
        drone_angle,
        drone_angle_vel,
        # rotation,
        lin_pids=None,
        rot_pids=None,
    ):
        self.time_step += 1
        print(f"self.time_step: {self.time_step}")
        modulo_value = 30
        if self.time_step % modulo_value <= int(modulo_value / 2):

            thrust = np.array([[0.3], [0.5], [0.5], [0.3]])
        else:
            thrust = np.array([[0.5], [0.3], [0.3], [0.5]])
        return thrust


class PIDControlUNnit(ControlUnit):
    def __init__(self, pids: list, quadcopter: QuadcopterPhysics):
        self.qc = quadcopter
        pass

    def _calc_pid_outputs(
        self, lab_pos, lab_lin_vel, drone_angle, drone_angle_vel, lin_pids, rot_pids
    ):

        # now, everything should be returned in lab coordinates
        # lab_pos, lab_lin_vel, drone_angle, drone_angle_vel = get_positions_and_angles(
        #     sensors
        # )

        # PID
        # pid_outputs = np.zeros([6, 1])
        # update linear
        lin_inputs = lab_pos
        lin_outputs = np.array(
            [
                [
                    pid.calculate(lin_inputs[i, 0], lab_lin_vel[i, 0])
                    for i, pid in enumerate(lin_pids)
                ]
            ]
        ).T

        # transform
        pid_outputs = np.zeros([6, 1])
        rot_outputs = np.zeros((3, 1))
        pid_outputs[0:3] = rot_outputs
        pid_outputs[3:6] = lin_outputs

        # new Setpoint
        rot_pids[0].set_setpoint(-pid_outputs[4])
        rot_pids[1].set_setpoint(pid_outputs[3])

        # update rotational
        rot_inputs = drone_angle
        rot_outputs = np.array(
            [
                [
                    pid.calculate(rot_inputs[i, 0], drone_angle_vel[i, 0])
                    for i, pid in enumerate(rot_pids)
                ]
            ]
        ).T
        pid_outputs[0:3] = rot_outputs
        pid_outputs[3:6] = lin_outputs

        return pid_outputs

    def _translate_pid_to_thrust(self, pid_outputs, rotation) -> np.ndarray:
        """
        :param pid_outputs 6 rows 1 column (3 rotation, 3 linear)
        :param rotation [roll, pitch, yaw]
        :param qc: instance of quadcopter physics
        :return: 4 rows 1 column
        """

        self.qc.Rot = rotation_matrix(rotation[0, 0], rotation[1, 0], rotation[2, 0])

        # desired_translation = pid_outputs[3, 0] * 0.5 * VEC_PITCH + \
        #     pid_outputs[4, 0] * 0.5 * VEC_ROLL

        desired_rotation = self.qc.angle_control_to_thrust.dot(pid_outputs[:3])

        base_multiplicator = self.qc.G / (4 * self.qc.c_f) + pid_outputs[5, 0]
        base_thrust = np.ones([4, 1]) * base_multiplicator

        try:
            transforming_ratio = 1 / (math.cos(rotation[0]) * math.cos(rotation[1]))
        except ZeroDivisionError:
            transforming_ratio = 1

        base_thrust_transformed = base_thrust * transforming_ratio
        base_thrust_transformed[base_thrust_transformed > 1] = 1
        base_thrust_transformed[base_thrust_transformed < 0] = 0

        thrust = base_thrust_transformed + desired_rotation
        thrust[thrust > 1] = 1
        thrust[thrust < 0] = 0

        return thrust

    def translate_input_to_thrust(
        self,
        lab_pos,
        lab_lin_vel,
        drone_angle,
        drone_angle_vel,
        rotation,
        lin_pids=None,
        rot_pids=None,
    ):
        pid_outputs = self._calc_pid_outputs(
            lab_pos, lab_lin_vel, drone_angle, drone_angle_vel, lin_pids, rot_pids
        )
        thrust = self._translate_pid_to_thrust(pid_outputs, rotation)
        return thrust
