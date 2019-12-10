"""
Created by Jan Schiffeler at 06.12.19
jan.schiffeler[at]gmail.com

Changed by



Python 3.6.5

"""

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
coef_force = 2  # N
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
ki = kp/ti
kd = kp * td
limitRange = [0, 100]
