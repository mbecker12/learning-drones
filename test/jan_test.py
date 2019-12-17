"""
Created by Jan Schiffeler at 14.12.19
jan.schiffeler[at]gmail.com

Changed by



Python 3.
Library version:


"""

# external
import numpy as np
from time import sleep

# parameters
from parameters import *

# classes
from PID import PID
from sensor import Sensor
from physical_model import QuadcopterPhysics

control_translations = False

initial_pos = np.array([[0.0, 0.0, 10.0]]).T
initial_angle = np.array([[0, 30, 0]]).T * np.pi / 180

initial_angle_vel = np.array([[0, 0, 0]]).T
initial_vel = np.array([[0.0, 0.0, 0.0]]).T

initial_wind_speed = np.array([[0.0, 0.0, 0.0]]).T

delta_t = delta_time
initial_thrust = np.array([[0.5, 0.5, 0.5, 0.5]]).T

x_target, y_target, z_target = 0, 0, 0
roll_target, pitch_target, yaw_target = 0 * np.pi/180, 0 * np.pi/180, 0 * np.pi/180

# initialization PID
rot_pids = [
    PID(kp=1., ki=0.0, kd=0.1, timeStep=delta_t, setValue=roll_target, integralRange=2, calculateFlag="rangeExit"),
    PID(kp=1., ki=0.0, kd=0.1, timeStep=delta_t, setValue=pitch_target, integralRange=2, calculateFlag="rangeExit"),
    PID(kp=1., ki=0.0, kd=0.1, timeStep=delta_t, setValue=yaw_target, integralRange=2, calculateFlag="rangeExit")
]

lin_pids = [
    PID(kp=1.0, ki=0.0, kd=0.9, timeStep=delta_t, setValue=x_target, integralRange=2, calculateFlag="rangeExit",
        outputLimitRange=[-1.57, 1.57]),
    PID(kp=1.5, ki=0.0, kd=0.3, timeStep=delta_t, setValue=y_target, integralRange=2, calculateFlag="rangeExit",
        outputLimitRange=[-1.57, 1.57]),
    PID(kp=1.5, ki=0.0, kd=0.3, timeStep=delta_t, setValue=z_target, integralRange=2, calculateFlag="rangeExit")
]

# initialization Sensor
sensors = [Sensor(delta_t, initial_pos[i, 0], initial_vel[i, 0]) for i in range(3)]
sensors.extend([Sensor(delta_t, initial_angle[i, 0], initial_angle_vel[i, 0]) for i in range(3)])

# initialization quadcopter
quadcopter = QuadcopterPhysics(
    mass_center=mass_center,
    mass_motor=mass_motor,
    radius_motor_center=radius_motor_center,
    coef_force=coef_force,
    coef_moment=coef_moment,
    coef_wind=coef_wind,
    gravity=gravity,
    mass_payload=mass_payload,
    x_payload=x_payload,
    y_payload=y_payload
)

# Load initial values to the process values
wind_speed = initial_wind_speed
lab_pos = initial_pos
lab_lin_vel = initial_vel
drone_angle = initial_angle
drone_angle_vel = initial_angle_vel
thrust = initial_thrust


# LOOP
for i in range(1000):
    sleep(0.05)

    # get
    lab_lin_acc, lab_rot_acc = quadcopter.calculate_accelerations(rotation=drone_angle, wind_speed=wind_speed,
                                                                  thrust=thrust, lin_acc_drone_2_lab=True)

    # update sensors
    [sensors[i].measure_acceleration(lab_lin_acc[i, 0]) for i in range(3)]
    [sensors[i].measure_acceleration(lab_rot_acc[i - 3, 0]) for i in range(3, 6)]

    # update PID
    pid_outputs = np.zeros([6, 1])

    # update linear
    lin_inputs = lab_pos
    lin_outputs = np.array([[pid.calculate(lin_inputs[i, 0]) for i, pid in enumerate(lin_pids)]]).T

    # new Setpoint
    rot_pids[0].set_setpoint(-pid_outputs[4])
    rot_pids[1].set_setpoint(pid_outputs[3])

    # update rotational
    rot_inputs = drone_angle
    rot_outputs = np.array([[pid.calculate(rot_inputs[i, 0]) for i, pid in enumerate(rot_pids)]]).T
    pid_outputs[0:3] = rot_outputs
    pid_outputs[3:6] = lin_outputs
    thrust = quadcopter.calculate_motor_thrust(pid_outputs, drone_angle)