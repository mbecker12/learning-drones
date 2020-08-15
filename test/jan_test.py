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
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# parameters
from parameters import *

# classes
from PID import PID
from sensor import Sensor, get_positions_and_angles
from physical_model import QuadcopterPhysics
from data_scraper import DataHandler

control_translations = False
visualize = False

# Set inital positions here!
initial_pos = np.array([[0.0, 0.0, 10.0]]).T
initial_angle = np.array([[0, 0, 0]]).T * np.pi / 180

initial_angle_vel = np.array([[0, 0, 0]]).T
initial_vel = np.array([[0.0, 0.0, 0.0]]).T

initial_wind_speed = np.array([[0.0, 0.0, 0.0]]).T

initial_thrust = np.array([[0.5, 0.5, 0.5, 0.5]]).T

x_target, y_target, z_target = 0, 0, 0
roll_target, pitch_target, yaw_target = (
    0 * np.pi / 180,
    60 * np.pi / 180,
    0 * np.pi / 180,
)


def update_pid(
    lin_inputs: np.ndarray,
    rot_inputs: np.ndarray,
    l_pids: list,
    r_pids: list,
    control_trans: bool,
):
    """
    update all PIDs
    """
    pid_outputs = np.zeros([6, 1])

    # update linear
    if control_trans:
        lin_outputs = np.array(
            [[pid.calculate(lin_inputs[i, 0]) for i, pid in enumerate(l_pids)]]
        ).T

        # new Setpoint
        rot_pids[0].set_setpoint(-pid_outputs[4])
        rot_pids[1].set_setpoint(pid_outputs[3])
        pid_outputs[3:6] = lin_outputs

    # update rotational
    rot_outputs = np.array(
        [[pid.calculate(rot_inputs[i, 0]) for i, pid in enumerate(r_pids)]]
    ).T
    pid_outputs[0:3] = rot_outputs

    return pid_outputs


def update_sensors(
    sensor: [Sensor], accelerations: np.ndarray
) -> (np.ndarray, np.ndarray):
    for i in range(accelerations.shape[0]):
        sensor[i].measure_acceleration(accelerations[i, 0])
    pos, lab_lin_vel, angle, drone_angle_vel = get_positions_and_angles(sensors)
    return pos, angle


# initialization PID
rot_pids = [
    PID(
        kp=0.7,
        ki=0.01,
        kd=0.1,
        timeStep=delta_time,
        setValue=roll_target,
        integralRange=2,
        calculateFlag="signChange",
        outputLimitRange=[-np.pi / 4, np.pi / 4],
    ),
    PID(
        kp=0.7,
        ki=0.01,
        kd=0.1,
        timeStep=delta_time,
        setValue=pitch_target,
        integralRange=2,
        calculateFlag="signChange",
        outputLimitRange=[-np.pi / 4, np.pi / 4],
    ),
    PID(
        kp=0.7,
        ki=0.001,
        kd=0.1,
        timeStep=delta_time,
        setValue=yaw_target,
        integralRange=2,
        calculateFlag="signChange",
        outputLimitRange=[-np.pi / 6, np.pi / 6],
    ),
]

lin_pids = [
    PID(
        kp=0.9,
        ki=0.1,
        kd=0.8,
        timeStep=delta_time,
        setValue=x_target,
        integralRange=2,
        calculateFlag="signChange",
        outputLimitRange=[-np.pi / 4, np.pi / 4],
    ),
    PID(
        kp=0.9,
        ki=0.1,
        kd=0.8,
        timeStep=delta_time,
        setValue=y_target,
        integralRange=2,
        calculateFlag="signChange",
        outputLimitRange=[-np.pi / 4, np.pi / 4],
    ),
    PID(
        kp=0.8,
        ki=0.1,
        kd=0.6,
        timeStep=delta_time,
        setValue=z_target,
        integralRange=2,
        calculateFlag="signChange",
    ),
]

# initialization Sensor
sensors = [Sensor(delta_time, initial_pos[i, 0], initial_vel[i, 0]) for i in range(3)]
sensors.extend(
    [Sensor(delta_time, initial_angle[i, 0], initial_angle_vel[i, 0]) for i in range(3)]
)

# initialization Quadcopter
quadcopter = QuadcopterPhysics(
    mass_center=mass_center,
    mass_motor=mass_motor,
    radius_motor_center=radius_motor_center,
    I_x=I_x,
    I_y=I_y,
    I_z=I_z,
    coef_force=coef_force,
    coef_moment=coef_moment,
    coef_wind=coef_wind,
    gravity=gravity,
    mass_payload=mass_payload,
    x_payload=x_payload,
    y_payload=y_payload,
)

# initialize data scraper
dh = DataHandler(
    parentfolder="results",
    visualize=visualize,
    n_servers=2,
    port=65432,
    printouts=False,
)

# Load initial values to the process values
wind_speed = initial_wind_speed
lab_pos = initial_pos
lab_lin_vel = initial_vel
drone_angle = initial_angle
drone_angle_vel = initial_angle_vel
thrust = initial_thrust


# LOOP
for time in range(1000):
    sleep(0.1)
    if time % 20 == 0:
        dh.new_setpoints(
            rotation=np.array([[roll_target, pitch_target, yaw_target]]).T,
            translation=np.array([[x_target, y_target, z_target]]).T,
        )

    # get acceleration
    lab_lin_acc, lab_rot_acc = quadcopter.calculate_accelerations(
        rotation=drone_angle,
        wind_speed=wind_speed,
        thrust=thrust,
        lin_acc_drone_2_lab=True,
    )

    # update sensors
    lab_pos, drone_angle = update_sensors(
        sensor=sensors, accelerations=np.concatenate([lab_lin_acc, lab_rot_acc])
    )

    # update PID
    pid_out = update_pid(
        lin_inputs=lab_pos,
        rot_inputs=drone_angle,
        l_pids=lin_pids,
        r_pids=rot_pids,
        control_trans=control_translations,
    )
    # print("PID: ", pid_out.T)
    if control_translations:
        roll_target = lin_pids[1].setValue
        pitch_target = lin_pids[0].setValue

    # update Thrust
    thrust = quadcopter.calculate_motor_thrust(pid_out, drone_angle)

    # save data
    dh.new_data(
        time=time * delta_time,
        rotation=drone_angle,
        translation=lab_pos,
        thrusters=thrust,
        wind=wind_speed,
        pid=pid_out,
    )

    print(
        "Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(
            drone_angle[0, 0] * 180 / np.pi,
            drone_angle[1, 0] * 180 / np.pi,
            drone_angle[2, 0] * 180 / np.pi,
        )
    )

dh.finish()
