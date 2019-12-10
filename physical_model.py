"""
Created by Jan Schiffeler at 01.12.19
jan.schiffeler[at]gmail.com

Changed by

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
"""

import numpy as np
from parameters import *
I_X_INV = 1.0 / I_x
I_Y_INV = 1.0 / I_y
I_Z_INV = 1.0 / I_z
VEC_ROLL = np.array([[1, -1, -1, 1]])
VEC_PITCH = np.array([[1, 1, -1, -1]])
VEC_YAW = np.array([[1, -1, 1, -1]])
from time import sleep
def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Calculate the 3D rotation matrix
    :param roll: rot around x
    :param pitch: rot around y
    :param yaw: rot around z
    :return: Rotation matrix of order Z,Y,X
    """
    R_x = np.array([[1,            0,                 0],
                    [0, np.cos(roll), -1 * np.sin(roll)],
                    [0, np.sin(roll),      np.cos(roll)]])

    R_y = np.array([[     np.cos(pitch), 0, np.sin(pitch)],
                    [                 0, 1,             0],
                    [-1 * np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -1 * np.sin(yaw), 0],
                    [np.sin(yaw),      np.cos(yaw), 0],
                    [          0,                0, 1]])

    return np.dot(R_z, np.dot(R_y, R_x))


class QuadcopterPhysics:
    """
    Class to calculate the forces and moments of the quadcopter based on the physical parameters
    Simplifications: - Direct control of motor thrust
                     - Direct resulting moment of motor thrust
                     - wind only from positive y-direction to negative y-direction
    """
    def __init__(self, mass_center: float, mass_motor: float,
                 radius_motor_center: float,
                 coef_force: float, coef_moment: float, coef_wind: float, gravity: float = 9.81,
                 mass_payload: float = 0, x_payload: float = 0, y_payload: float = 0):
        self.m_c = mass_center
        self.m_m = mass_motor
        self.m_p = mass_payload
        self.r_m = radius_motor_center
        self.c_f = coef_force
        self.c_m = coef_moment
        self.c_w = coef_wind
        self.G = gravity * (4 * self.m_m + self.m_c + self.m_p)
        self.moments_payload = np.zeros([3, 1])

        if mass_payload != 0:
            force_payload = gravity * mass_payload
            self.G += force_payload
            self.moments_payload[:, 0] = np.array([[x_payload * force_payload, y_payload * force_payload, 0]])

    def calculate_forces_and_moments(self, thrust: np.ndarray,
                                     roll: float, pitch: float, yaw: float,
                                     wind_speed: float) -> (np.ndarray, np.ndarray):
        """
        name is pretty obvious don't you think?
        :param thrust: [[T_0], [T_1], [T_2], [T_3]]
        :param roll: rot around x
        :param pitch: rot around y
        :param yaw: rot around z
        :param wind_speed:
        :return: (forces vector [[X], [Y], [Z]], moments vector [[L],[M],[N]]
        """
        # calculate current rotations, environment influences, forces and moments of the rotors
        Rot = rotation_matrix(roll, pitch, yaw)
        T = thrust * self.c_f
        W = wind_speed * self.c_w
        R = thrust * self.c_m

        # resulting forces
        forces = np.array([[0, 0, np.sum(T)]], dtype=np.float32).T
        G_rotated = np.dot(Rot, np.array([[0, 0, -1 * self.G]], dtype=np.float32).T)
        W_rotated = np.dot(Rot, np.array([[0, -1 * W, 0]], dtype=np.float32).T)
        forces += G_rotated + W_rotated

        # resulting moments
        L = self.r_m * (T[0, 0] + T[0, 3] - T[0, 1] - T[0, 2])
        M = self.r_m * (T[0, 0] + T[0, 1] - T[0, 2] - T[0, 3])
        N = np.sqrt(2) * self.r_m * (R[0, 1] + R[0, 3] - R[0, 0] - R[0, 2])
        moments = np.array([[L, M, N]]).T
        moments += self.moments_payload

        return forces, moments

    def convert_to_acceleration(self,
                                forces: np.ndarray,
                                moments: np.ndarray) -> (np.ndarray, np.ndarray):
        """
        Convert the forces and momenta to useful linear and rotational
        acceleration, resepectively.
        :param forces: calculated linear forces
        :param moments: calculated rotational moments
        :return: (linear acceleration, rotational acceleration)
        """
        lin_acc = np.divide(forces, self.m_c + 4 * self.m_m + self.m_p)
        rot_acc = np.array([
            moments[0] * I_X_INV,
            moments[1] * I_Y_INV,
            moments[2] * I_Z_INV])

        return lin_acc, rot_acc

    def controll_thrust(self,
                        pid_outputs: np.ndarray,
                        roll: float,
                        pitch: float,
                        yaw: float,
                        delta_z: float) -> np.ndarray:
        """
        From the PID outputs, calculate useful thrust levels for all
        four rotors.
        :param pid_outputs: outputs of three pid
        :param roll: current roll
        :param pitch: current pitch
        :param yaw: current yaw
        :param delta_z: desired change in height (in lab coordinates)
        :return: thrust levels
        """
        print(pid_outputs)
        desired_roll = pid_outputs[0] * VEC_ROLL
        desired_pitch = pid_outputs[1] * VEC_PITCH
        desired_yaw = pid_outputs[2] * VEC_YAW

        thrust = np.array(desired_roll + desired_pitch + desired_yaw)
        print(thrust)
        sleep(1)
        vec_sum = np.sum(thrust)
        if vec_sum != 0:
            thrust /= vec_sum
        print(thrust, vec_sum)
        sleep(1)
        Rot = rotation_matrix(roll, pitch, yaw)
        G_rotated = np.dot(Rot, np.array([[0, 0, -1 * self.G + delta_z]], dtype=np.float32).T)    
        print(G_rotated)
        sleep(1)
        ratio = self.c_f / G_rotated[2, 0]
        print(ratio)
        thrust *= ratio
        print(thrust)
        return thrust


if __name__ == "__main__":
    qc = QuadcopterPhysics(mass_center=1, mass_motor=0.25, radius_motor_center=0.5,
                           coef_force=1, coef_moment=1, coef_wind=1, gravity=10, mass_payload=1, x_payload=1)
    t = np.array([[1, 1, 1, 1]])
    f, m = qc.calculate_forces_and_moments(thrust=t, roll=0, pitch=0, yaw=0, wind_speed=0)
    print(f)
    print(m)
