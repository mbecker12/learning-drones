import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
import numpy as np
from physical_model import rotation_matrix, QuadcopterPhysics, translational_matrix
from PID import PID
from parameters import *
from data_scraper import DataHandler
from sensor import Sensor, get_positions_and_angles
from time import sleep
from utility import *
import sys


thrust_type = "pitch"

if __name__ == "__main__":
    if len(sys.argv) >= 1:
        port = int(sys.argv[1])
    else:
        port = 65432

    if len(sys.argv) >= 3:
        visualize = bool(int(sys.argv[2]))
    else:
        visualize = True

    if len(sys.argv) >= 4:
        n_servers = int(sys.argv[3])
    else:
        n_servers = 2

    # initialize drone, choose starting position
    initial_thrust = np.array([[0, 0, 0, 0]]).T
    initial_wind_speed = np.array([[0.0, 0.0, 0.0]]).T
    initial_angle = np.array([[0, 0, 0]]).T * np.pi / 180
    initial_angle_vel = np.array([[0, 0, 0]]).T
    initial_pos = np.array([[0.0, 0.0, 10.0]]).T
    initial_vel = np.array([[0.0, 0.0, 0.0]]).T
    delta_t = 0.01
    timesteps = 5000

    x_target = 0
    y_target = 0
    z_target = 10
    roll_target = 0
    pitch_target = 30
    yaw_target = 0
    # yaw_target = np.arctan(y_target / x_target) * 180 / np.pi
    print(f"yaw_target: {yaw_target}")
    rot_outputs = np.array([[roll_target, pitch_target, yaw_target]]).T * np.pi / 180
    lin_outputs = np.array([[x_target, y_target, z_target]]).T
    lin_targets = np.array([[x_target, y_target, z_target]]).T
    rot_targets = np.array([[roll_target, pitch_target, yaw_target]]).T * np.pi / 180
    # Initialize Drone Hardware
    dh = DataHandler(
        parentfolder="results", visualize=visualize, n_servers=n_servers, port=port
    )
    sensors = [Sensor(delta_t, initial_pos[i, 0], initial_vel[i, 0]) for i in range(3)]
    sensors.extend(
        [
            Sensor(delta_t, initial_angle[i, 0], initial_angle_vel[i, 0])
            for i in range(3)
        ]
    )

    rot_pids = [
        PID(
            kp=0.7,
            ki=0.1,
            kd=0.1,
            timeStep=delta_t,
            setValue=roll_target * np.pi / 180,
            integralRange=2,
            calculateFlag="signChange",
            outputLimitRange=[-np.pi / 4, np.pi / 4],
        ),
        PID(
            kp=0.7,
            ki=0.1,
            kd=0.1,
            timeStep=delta_t,
            setValue=pitch_target * np.pi / 180,
            integralRange=2,
            calculateFlag="signChange",
            outputLimitRange=[-np.pi / 4, np.pi / 4],
        ),
        PID(
            kp=0.2,
            ki=0.01,
            kd=0.1,
            timeStep=delta_t,
            setValue=yaw_target * np.pi / 180,
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
            timeStep=delta_t,
            setValue=x_target,
            integralRange=2,
            calculateFlag="signChange",
            outputLimitRange=[-np.pi / 5, np.pi / 5],
        ),
        PID(
            kp=0.9,
            ki=0.1,
            kd=0.8,
            timeStep=delta_t,
            setValue=y_target,
            integralRange=2,
            calculateFlag="signChange",
            outputLimitRange=[-np.pi / 5, np.pi / 5],
        ),
        PID(
            kp=0.8,
            ki=0.1,
            kd=0.6,
            timeStep=delta_t,
            setValue=z_target,
            integralRange=2,
            calculateFlag="signChange",
        ),
    ]

    quadcopter = QuadcopterPhysics(
        mass_center=mass_center,
        mass_motor=mass_motor,
        radius_motor_center=radius_motor_center,
        coef_force=coef_force,
        coef_moment=coef_moment,
        coef_wind=coef_wind,
        gravity=gravity,
        mass_payload=0.0,
        x_payload=0.0,
        y_payload=0.0,
        I_x=I_x,
        I_y=I_y,
        I_z=I_z,
    )

    # Initialize Values
    wind_speed = initial_wind_speed
    lab_pos = initial_pos
    lab_lin_vel = initial_vel
    drone_angle = initial_angle
    drone_angle_vel = initial_angle_vel
    thrust = initial_thrust

    forces, moments = quadcopter.calculate_forces_and_moments(
        thrust=initial_thrust,
        roll=initial_angle[0, 0],
        pitch=initial_angle[1, 0],
        yaw=initial_angle[2, 0],
        wind_speed=wind_speed,
    )

    # force changes
    ###############################################
    ################ MAIN LOOP ####################
    ###############################################
    target_cheoreography = np.asarray(
        [
            [[0.52], [0.0], [0.0], [0.0], [0.0], [10.0]],
            [[0.52], [0.52], [0.0], [0.0], [0.0], [10.0]],
            [[0.0], [0.0], [0.0], [0.0], [0.0], [10.0]],
            [[0.0], [0.0], [1.0], [0.0], [0.0], [10.0]],
            [[0.0], [0.0], [0.0], [0.0], [0.0], [10.0]],
            [[0.52], [0.0], [0.3], [0.0], [0.0], [10.0]],
            [[0.52], [0.0], [0.5], [0.0], [0.0], [10.0]],
            [[0.0], [0.0], [0.5], [0.0], [0.0], [10.0]],
            [[0.0], [0.3], [0.5], [0.0], [0.0], [10.0]],
            [[0.0], [0.0], [0.0], [0.0], [0.0], [10.0]],
            [[0.0], [0.7], [0.0], [0.0], [0.0], [10.0]],
            [[0.52], [0.7], [0.0], [0.0], [0.0], [10.0]],
        ]
    )
    stab_counter = 0
    stab_time_max = 60
    target_index = 0
    for time in range(timesteps):
        current = np.concatenate([drone_angle, lab_pos])
        targets = np.concatenate([rot_targets, lin_targets])
        radius = 2
        stab = stability_check(targets, current, roll=0.1, pitch=0.1, yaw=0.1)
        print(f"stab: {stab}")
        stab_counter, target_index = check_and_count_stability(
            stab, stab_time_max, stab_counter, target_index
        )
        rot_targets = target_cheoreography[target_index][0:3]
        lin_targets = target_cheoreography[target_index][3:6]
        rot_pids[0].set_setpoint(rot_targets[0, 0])
        rot_pids[1].set_setpoint(rot_targets[1, 0])
        rot_pids[2].set_setpoint(rot_targets[2, 0])

        lin_pids[0].set_setpoint(lin_targets[0, 0])
        lin_pids[1].set_setpoint(lin_targets[1, 0])
        lin_pids[2].set_setpoint(lin_targets[2, 0])
        # rot_pids, lin_pids = update_setpoints(target_cheoreography, target_index, rot_pids, lin_pids)
        print(f"stab_counter: {stab_counter}, target_index: {target_index}")
        real_time = time * delta_t
        changed_setpoint = False
        if time % 20 == 0:
            changed_setpoint = True

        if changed_setpoint:
            dh.new_setpoints(rot_targets, lin_targets)

        try:
            sleep(0.05)
        except KeyboardInterrupt:
            dh.finish()

        # calculate accelerations
        lab_lin_acc, lab_rot_acc = quadcopter.calculate_accelerations(
            drone_angle, wind_speed, thrust, lin_acc_drone_2_lab=True
        )

        print(f"lab_lin_acc: {lab_lin_acc}")
        # Tr = translational_matrix(drone_angle[0, 0], drone_angle[1, 0], drone_angle[2, 0])
        # lab_rot_acc = np.dot(Tr, drone_rot_acc)

        # measurement in lab frame
        [sensors[i].measure_acceleration(lab_lin_acc[i, 0]) for i in range(3)]
        [sensors[i].measure_acceleration(lab_rot_acc[i - 3, 0]) for i in range(3, 6)]

        # now, everything should be returned in lab coordinates
        lab_pos, lab_lin_vel, drone_angle, drone_angle_vel = get_positions_and_angles(
            sensors
        )

        print(f"x: {lab_pos[0, 0]}, vx: {lab_lin_vel[0, 0]}, ax: {lab_lin_acc[0, 0]}")
        print(f"y: {lab_pos[1, 0]}, vy: {lab_lin_vel[1, 0]}, ay: {lab_lin_acc[1, 0]}")
        print(f"z: {lab_pos[2, 0]}, vz: {lab_lin_vel[2, 0]}, az: {lab_lin_acc[2, 0]}")

        # PID
        pid_outputs = np.zeros([6, 1])
        # update linear
        lin_inputs = lab_pos
        lin_outputs = np.array(
            [[pid.calculate(lin_inputs[i, 0]) for i, pid in enumerate(lin_pids)]]
        ).T

        # transform

        pid_outputs[0:3] = rot_outputs
        pid_outputs[3:6] = lin_outputs
        # yaw_world, pid_outputs = quadcopter.translate_rotation_to_global(drone_angle, pid_outputs)

        # new Setpoint
        # rot_pids[0].set_setpoint(-pid_outputs[4])
        # rot_pids[1].set_setpoint(pid_outputs[3])

        # if np.abs(pid_outputs[3]) < 0.1 and np.abs(pid_outputs[4]) < 0.1:
        #     yaw_target = np.arctan(y_target / x_target)
        #     rot_pids[2].set_setpoint(yaw_target)
        print(f"pid_outputs: {pid_outputs}")

        # update rotational
        rot_inputs = drone_angle
        rot_outputs = np.array(
            [[pid.calculate(rot_inputs[i, 0]) for i, pid in enumerate(rot_pids)]]
        ).T
        pid_outputs[0:3] = rot_outputs
        pid_outputs[3:6] = lin_outputs
        print(f"pid_outputs: {pid_outputs}")
        thrust = quadcopter.calculate_motor_thrust(pid_outputs, drone_angle)
        # thrust = quadcopter.control_thrust(
        #     rot_outputs, drone_angle[0, 0], drone_angle[1, 0], drone_angle[2, 0],
        #   pid_outputs[5, 0], delta_x=pid_outputs[3, 0], delta_y=pid_outputs[4, 0])

        dh.new_data(
            time=time * delta_t,
            rotation=drone_angle,
            translation=lab_pos,
            thrusters=thrust,
            wind=wind_speed,
            pid=pid_outputs,
        )

        forces, moments = quadcopter.calculate_forces_and_moments(
            thrust=thrust,
            roll=drone_angle[0, 0],
            pitch=drone_angle[1, 0],
            yaw=drone_angle[2, 0],
            wind_speed=wind_speed,
        )

    dh.finish()
