"""
Provide rotation matrices and other mathematical necessities
"""
import numpy as np
from drone.quadcopter.parameters import I_x, I_y, I_z

I_X_INV = 1.0 / I_x
I_Y_INV = 1.0 / I_y
I_Z_INV = 1.0 / I_z
VEC_ROLL = np.array([[1, -1, -1, 1]]).T
VEC_PITCH = np.array([[-1, -1, 1, 1]]).T
VEC_YAW = np.array([[-1, 1, -1, 1]]).T


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Calculate the 3D rotation matrix
    :param roll: rot around x
    :param pitch: rot around y
    :param yaw: rot around z
    :return: Rotation matrix of order Z,Y,X
    """
    R_x = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -1 * np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ]
    )

    R_y = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-1 * np.sin(pitch), 0, np.cos(pitch)],
        ]
    )

    R_z = np.array(
        [[np.cos(yaw), -1 * np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
    )

    return np.dot(R_z, np.dot(R_y, R_x)).T
    # return np.dot(R_x, np.dot(R_y, R_z))


def rotation_matrix_2d(alpha):
    rota = np.array([[np.cos(alpha), np.sin(alpha)], [-np.sin(alpha), np.cos(alpha)]])
    return rota


def translational_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Calculate the rotational matrix to convert angular velocities 
    from drone-frame to inertial frame.
    :param roll: rot around x
    :param pitch: rot around y
    :param yaw: rot around z
    :return: translational matrix
    """

    transl = np.array(
        [
            [1, np.sin(roll) * np.tan(pitch), np.cos(roll) * np.tan(pitch)],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll) / np.cos(pitch), np.cos(roll) / np.cos(pitch)],
        ],
        dtype=np.float32,
    )

    return transl
