"""
Created by Jan Schiffeler at 01.12.19
jan.schiffeler[at]gmail.com

Changed by
Marvin Becker

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
import math
from drone.quadcopter.parameters import *
from drone.quadcopter.geometry import *
from time import sleep


class QuadcopterPhysics:
    """
    Class to calculate the forces and moments of the quadcopter based on the physical parameters
    Simplifications: - Direct control of motor thrust
                     - Direct resulting moment of motor thrust
                     - wind only from positive y-direction to negative y-direction
    """

    def __init__(
        self,
        mass_center: float,
        mass_motor: float,
        radius_motor_center: float,
        I_x: float,
        I_y: float,
        I_z: float,
        coef_force: float,
        coef_moment: float,
        coef_wind: float,
        gravity: float = 9.81,
        mass_payload: float = 0,
        x_payload: float = 0,
        y_payload: float = 0,
    ):
        self.m_c = mass_center
        self.m_m = mass_motor
        self.m_p = mass_payload
        self.r_m = radius_motor_center
        self.c_f = coef_force
        self.c_m = coef_moment
        self.c_w = coef_wind
        self.Rot = np.identity(3, dtype=np.float32)
        self.G = gravity * (4 * self.m_m + self.m_c)
        self.moments_payload = np.zeros([3, 1])
        self.inverse_inertia_vector = np.array([[1 / I_x, 1 / I_y, 1 / I_z]]).T
        self.angle_control_to_thrust = np.array(
            [[1, -1, -1], [-1, -1, 1], [-1, 1, -1], [1, 1, 1]]
        )

        if mass_payload != 0:
            force_payload = gravity * mass_payload
            self.G += force_payload
            self.moments_payload[:, 0] = np.array(
                [[x_payload * force_payload, y_payload * force_payload, 0]]
            )

    def calculate_accelerations(
        self,
        rotation: np.ndarray,
        wind_speed: np.ndarray,
        thrust: np.ndarray,
        lin_acc_drone_2_lab: bool,
    ) -> (np.ndarray, np.ndarray):
        """
        Calculate linear and rotational acceleration of the instantaneous state.
        :param rotation [roll, pitch, yaw]
        :param wind_speed    3 rows 1 column
        :param thrust    4 rows 1 column
        :param lin_acc_drone_2_lab
        :return: 3 rows 1 column each
        """
        self.Rot = rotation_matrix(rotation[0, 0], rotation[1, 0], rotation[2, 0])
        # print(f"rotation matrix: {self.Rot}")
        # motor forces
        T = thrust * self.c_f
        R = thrust * self.c_m

        # resulting forces
        # thrust forces in drone coordinates
        forces = np.array([[0, 0, np.sum(T)]], dtype=np.float32).T
        # print(f"forces: {forces}")
        G_rotated = np.dot(
            self.Rot, np.array([[0, 0, -1 * self.G]], dtype=np.float32).T
        )
        W_rotated = np.dot(self.Rot, wind_speed.astype(np.float32) * self.c_w)
        forces += G_rotated + W_rotated

        # resulting moments
        L = self.r_m * (T[0, 0] + T[3, 0] - T[1, 0] - T[2, 0])
        M = self.r_m * (T[2, 0] + T[3, 0] - T[0, 0] - T[1, 0])
        N = np.sqrt(2) * self.r_m * (R[1, 0] + R[3, 0] - R[0, 0] - R[2, 0])
        moments = np.array([[L, M, N]]).T
        moments += self.moments_payload

        # accelerations
        lin_acc = forces / (self.m_c + 4 * self.m_m + self.m_p)
        rot_acc = moments * self.inverse_inertia_vector

        if lin_acc_drone_2_lab:
            lin_acc = np.dot(self.Rot.T, lin_acc)
            Tr = translational_matrix(rotation[0, 0], rotation[1, 0], rotation[2, 0])
            rot_acc = np.dot(Tr, rot_acc)

        return lin_acc, rot_acc


# if __name__ == "__main__":
#     from parameters import *

#     qc = QuadcopterPhysics(
#         mass_center=mass_center,
#         mass_motor=mass_motor,
#         radius_motor_center=radius_motor_center,
#         I_x=I_x,
#         I_y=I_y,
#         I_z=I_z,
#         coef_force=coef_force,
#         coef_moment=coef_moment,
#         coef_wind=coef_wind,
#         gravity=gravity,
#         mass_payload=mass_payload,
#         x_payload=x_payload,
#         y_payload=y_payload,
#     )
#     t = np.array([[0.1, 0.1, 0.1, 0.1]])
#     roll, pitch, yaw = 45 * 3.14159 / 180, 30 * 3.14159 / 180, 45 * 3.14159 / 180
#     wind = np.array([[0, 0, 0]])
#     # print(qc.calculate_accelerations(rotation=np.array([[roll, pitch, yaw]]).T, wind_speed=wind.T, thrust=t.T, lin_acc_drone_2_lab=False))
#     f, m = qc.calculate_forces_and_moments(
#         thrust=t, roll=roll, pitch=pitch, yaw=yaw, wind_speed=wind
#     )
#     # print(qc.convert_to_acceleration(f, m))
#     # print(f)
#     # print(m)

#     pid = np.array([[0.0, 0.0, 0.0, 1, 0.0, -0.2]], dtype=np.float32).T
#     t = qc.calculate_motor_thrust(
#         rotation=np.array([[roll, pitch, yaw]]).T, pid_outputs=pid
#     )
#     # print(t)
#     t = qc.control_thrust(
#         roll=roll, pitch=pitch, yaw=yaw, pid_outputs=pid[:3], delta_z=pid[5, 0]
#     )
#     # print(t)

#     angle, pid_new = qc.translate_rotation_to_global(
#         rotation=np.array([[roll, pitch, yaw]]).T, pid_outputs=pid
#     )
#     print(
#         "angle: {:.2f}, x: {:.2f}, y: {:.2f}".format(
#             angle * 180 / 3.14159, pid_new[3, 0], pid_new[4, 0]
#         )
#     )
