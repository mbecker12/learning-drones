import numpy as np
import math
from parameters import *

from physical_model import rotation_matrix, QuadcopterPhysics

roll = 30 * np.pi / 180
pitch = 10 * np.pi / 180
yaw = 45 * np.pi / 180

Rot = rotation_matrix(roll, pitch, yaw)

qc = QuadcopterPhysics(
    mass_center=mass_center,
    mass_motor=mass_motor,
    radius_motor_center=radius_motor_center,
    coef_force=coef_force,
    coef_moment=coef_moment,
    coef_wind=coef_wind,
    gravity=gravity,
    mass_payload=mass_payload,
    x_payload=x_payload,
    y_payload=y_payload,
)

# base force for hovering in level flight
base_thrust_force_drone = np.array([[0, 0, qc.G]], dtype=np.float32)
print(f"base_thrust_force_drone: {base_thrust_force_drone}")
base_thrust_force_lab = np.dot(base_thrust_force_drone, Rot.T)
print(f"base_thrust_force_lab: {base_thrust_force_lab}")
ratio = qc.G / base_thrust_force_lab[0, 2]
print(f"ratio: {ratio}")
print(f"1/cos({roll}): {1 / (np.cos(roll) * np.cos(pitch))}")
base_thrust_force_lab *= ratio
print(f"base_thrust_force_lab: {base_thrust_force_lab}")
new_thrust_drone = np.dot(base_thrust_force_lab, Rot)
print(f"new_thrust_drone: {new_thrust_drone}")

try:
    1 / 0
except ZeroDivisionError as zerror:
    print("caught ya")
