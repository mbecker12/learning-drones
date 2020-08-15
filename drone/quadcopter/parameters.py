"""
Created by Jan Schiffeler at 06.12.19
jan.schiffeler[at]gmail.com

Changed by
Marvin Becker


Python 3.6.5

"""

payload_set = None  # set to None if no payload
# GEOMETRY AND MASS
mass_center = 0.200  # kg
mass_motor = 0.050  # kg
radius_motor_center = 0.1  # m
I_x = 2160e-6  # kgm^2
I_y = 2160e-6  # kgm^2
I_z = 4210e-6  # kgm^2

# CONSTANTS
gravity = 9.81  # m/s^2
delta_time = 0.05  # s

# COEFFICIENTS
# TODO: force per motor or total force?
coef_force = 5  # N # 4 * c_f ~= mass_total * 2
coef_moment = 0.5  # N

# ENVIRONMENT
coef_wind = 0.1  # kg/m
mass_payload = 0  # kg
x_payload = 0  # m
y_payload = 0  # m

# PID Constants
kp = 1
ti = 1
td = 1
ki = kp / ti
kd = kp * td
limitRange = [0, 1]

if payload_set is not None:
    import numpy as np

    iom_collection = np.load("iom_collection.npy")
    mass_payload = iom_collection[payload_set, 0]  # kg
    x_payload = iom_collection[payload_set, 1]  # m
    y_payload = iom_collection[payload_set, 2]  # m
    I_x = iom_collection[payload_set, 3]  # kgm^2
    I_y = iom_collection[payload_set, 4]  # kgm^2
    I_z = iom_collection[payload_set, 5]  # kgm^2