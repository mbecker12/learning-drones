import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
import numpy as np
from drone.quadcopter.physical_model import (
    rotation_matrix,
    QuadcopterPhysics,
    translational_matrix,
)
from drone.controller.pid import PID
from drone.quadcopter.parameters import *
from datahandling.data_scraper import DataHandler
from drone.sensor.sensor import Sensor, get_positions_and_angles
from time import sleep
from drone.utility import *
from drone.quadcopter.choose_paramset import *
from drone.onboard_computer import PIDControlUNnit
import sys
from objects.coins import Coin


mode = "translation"
typ = "triple"
mode = mode + "/" + typ
if payload_set is not None:
    mode = mode + "/" + str(payload_set)
else:
    if "payload" in mode:
        mode = mode + "/" + "99"

if __name__ == "__main__":
    # print(f"sys.argv: {sys.argv}")
    if len(sys.argv) >= 1:
        port = int(sys.argv[1])
    else:
        port = 65432

    if len(sys.argv) >= 3:
        visualize = bool(int(sys.argv[2]))
    else:
        visualize = False

    if len(sys.argv) >= 4:
        n_servers = int(sys.argv[3])
    else:
        n_servers = 0

    if len(sys.argv) >= 5:
        paramset_num = sys.argv[4]
    else:
        paramset_num = "default"
    assert paramset_num in [
        "00",
        "01",
        "02",
        "10",
        "11",
        "12",
        "13",
        "20",
        "21",
        "22",
    ], str(paramset_num)
    paramset = get_paramset(paramset_num)
    (
        kp_x,
        kp_y,
        kp_z,
        ki_x,
        ki_y,
        ki_z,
        kd_x,
        kd_y,
        kd_z,
        kp_rol,
        kp_pit,
        kp_yaw,
        ki_rol,
        ki_pit,
        ki_yaw,
        kd_rol,
        kd_pit,
        kd_yaw,
    ) = convert_paramset_2_float(paramset)
    # initialize drone, choose starting position
    initial_thrust = np.array([[0, 0, 0, 0]]).T
    initial_wind_speed = np.array([[0.0, 0.0, 0.0]]).T
    initial_angle = np.array([[0, 0, 0]]).T * np.pi / 180
    initial_angle_vel = np.array([[0, 0, 0]]).T
    initial_pos = np.array([[0.0, 0.0, 10.0]]).T
    initial_vel = np.array([[0.0, 0.0, 0.0]]).T
    delta_t = 0.01
    timesteps = 300

    x_target = 10
    y_target = 10
    z_target = 10
    roll_target = 0
    pitch_target = 0
    yaw_target = 0
    # yaw_target = np.arctan(y_target / x_target) * 180 / np.pi
    print(f"yaw_target: {yaw_target}")
    rot_outputs = np.array([[roll_target, pitch_target, yaw_target]]).T * np.pi / 180
    lin_outputs = np.array([[x_target, y_target, z_target]]).T
    lin_targets = np.array([[x_target, y_target, z_target]]).T
    rot_targets = np.array([[roll_target, pitch_target, yaw_target]]).T * np.pi / 180
    # Initialize Drone Hardware
    dh = DataHandler(
        parentfolder=mode + "/" + paramset_num,
        visualize=visualize,
        n_servers=n_servers,
        port=port,
        # paramset=paramset,
        # payload_set=payload_set,
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
            kp=kp_rol,
            ki=ki_rol,
            kd=kd_rol,
            timeStep=delta_t,
            setValue=roll_target * np.pi / 180,
            integralRange=2,
            calculateFlag="signChange",
            outputLimitRange=[-np.pi / 4, np.pi / 4],
        ),
        PID(
            kp=kp_pit,
            ki=ki_pit,
            kd=kd_pit,
            timeStep=delta_t,
            setValue=pitch_target * np.pi / 180,
            integralRange=2,
            calculateFlag="signChange",
            outputLimitRange=[-np.pi / 4, np.pi / 4],
        ),
        PID(
            kp=kp_yaw,
            ki=ki_yaw,
            kd=kd_yaw,
            timeStep=delta_t,
            setValue=yaw_target * np.pi / 180,
            integralRange=2,
            calculateFlag="signChange",
            outputLimitRange=[-np.pi / 6, np.pi / 6],
        ),
    ]

    lin_pids = [
        PID(
            kp=kp_x,
            ki=ki_x,
            kd=kd_x,
            timeStep=delta_t,
            setValue=x_target,
            integralRange=2,
            calculateFlag="signChange",
            outputLimitRange=[-np.pi / 4, np.pi / 4],
        ),
        PID(
            kp=kp_y,
            ki=ki_y,
            kd=kd_y,
            timeStep=delta_t,
            setValue=y_target,
            integralRange=2,
            calculateFlag="signChange",
            outputLimitRange=[-np.pi / 4, np.pi / 4],
        ),
        PID(
            kp=kp_z,
            ki=ki_y,
            kd=kd_z,
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
        mass_payload=mass_payload,
        x_payload=x_payload,
        y_payload=y_payload,
        I_x=I_x,
        I_y=I_y,
        I_z=I_z,
    )

    controller = PIDControlUNnit(
        np.concatenate([lin_pids, rot_pids]),
        quadcopter
        )

    # Initialize Values
    wind_speed = initial_wind_speed
    lab_pos = initial_pos
    lab_lin_vel = initial_vel
    drone_angle = initial_angle
    drone_angle_vel = initial_angle_vel
    thrust = initial_thrust

    lab_lin_acc, lab_rot_acc = quadcopter.calculate_accelerations(
        drone_angle, wind_speed, thrust, lin_acc_drone_2_lab=True
    )

    pid_outputs = np.zeros([6, 1])
    dh.new_data(
        time=0 * delta_t,
        rotation=drone_angle,
        translation=lab_pos,
        thrusters=thrust,
        wind=wind_speed,
        pid=pid_outputs,
    )

    # force changes
    ###############################################
    ################ MAIN LOOP ####################
    ###############################################
    # roll, pitch, yaw (all in radians), x, y, z
    
    # visited points:

    # target_cheoreography = np.asarray([
    #     [[0], [0], [0], [0.0], [0.0], [5.0]],
    #     [[0], [0], [np.arctan(10./10)], [0.0], [0.0], [10.0]],
    #     [[0], [0], [np.arctan(10./10)], [10.0], [10.0], [10.0]],
    #     [[0], [0], [np.arctan((15-10.)/(5-10.))], [5.0], [15.0], [10.0]],
    #     [[0], [0], [np.arctan((0-15.)/(12-5.))], [12.0], [0.0], [10.0]],
    #     [[0], [0], [np.arctan((0-15.)/(12-5.))], [12.0], [0.0], [15.0]],
    #     [[0], [0], [np.arctan((0.)/(12-0.))], [0.0], [0.0], [15.0]],
    #     [[0], [0], [np.arctan((0.)/(12-0.))], [0.0], [0.0], [0.0]],
    #     ])

    target_cheoreography = np.asarray(
        [
            [[0], [0], [0], [0.0], [0.0], [5.0]],
            [[0], [0], [0], [5.0], [5.0], [5.0]],
            [[0], [0], [0], [2.0], [8.0], [5.0]],
            [[0], [0], [0], [8.0], [0.0], [5.0]],
            # [[0], [0], [0], [12.0], [0.0], [15.0]],
            [[0], [0], [0], [0.0], [0.0], [10.0]],
            [[0], [0], [0], [0.0], [0.0], [0.0]],
        ]
    )

    target_coins = [
        Coin(np.asarray([[0.0, 0.0, 5.0]]).T, 1, value=1000),
        Coin(np.asarray([[25.0, 25.0, 5.0]]).T, 1, value=1000),
        Coin(np.asarray([[-10.0, 35.0, 5.0]]).T, 1, value=1000),
        Coin(np.asarray([[0.0, 0.0, 5.0]]).T, 1, value=1000)
    ]

    stab_counter = 0
    stab_time_max = 50
    target_index = 0
    new_target_index = 0
    angle_tolerance = 0.05
    xy_tolerance_radius = 1.0
    z_tolerance = 0.5
    timesteps = 3500
    score = timesteps
    sleep(2)
    for time in range(timesteps):
        score -= 1
        current = np.concatenate([drone_angle, lab_pos])
        targets = np.concatenate([rot_targets, lin_targets])
        radius = 2

        collected_coin = target_coins[target_index].is_in_vicinity(lab_pos)
        if collected_coin:
            score += target_coins[target_index].value
            new_target_index = target_index + 1
            


        if new_target_index >= len(target_coins):
            print(f"Final Score: {score}, Paramset: {paramset_num}")
            dh.finish()
            sys.exit()

        rot_targets = np.zeros((3, 1))
        lin_targets = target_coins[new_target_index].position
        # rot_targets = target_cheoreography[new_target_index][0:3]
        # lin_targets = target_cheoreography[new_target_index][3:6]
        print(f"x_target: {lin_targets[-3]}")
        print(f"y_target: {lin_targets[-2]}")
        print(f"z_target: {lin_targets[-1]}")
        [rot_pids[i].set_setpoint(rot_targets[i, 0]) for i in range(3)]
        [lin_pids[i].set_setpoint(lin_targets[i, 0]) for i in range(3)]
        if new_target_index > target_index:
            dh.new_setpoints(rot_targets, lin_targets)
            dh.new_setpoints(rot_targets, lin_targets)
            dh.new_setpoints(rot_targets, lin_targets)
            print(f"new_target_index: {new_target_index}, target_index: {target_index}")
            target_index = new_target_index

        print(f"score: {score}, target_index: {target_index}")
        real_time = time * delta_t
        changed_setpoint = False
        if time % 10 == 0:
            changed_setpoint = True

        if changed_setpoint:
            dh.new_setpoints(rot_targets, lin_targets)

        try:
            if not visualize and n_servers < 2:
                pass
            else:
                sleep(0.1)
        except KeyboardInterrupt:
            dh.finish()

        # calculate accelerations
        lab_lin_acc, lab_rot_acc = quadcopter.calculate_accelerations(
            drone_angle, wind_speed, thrust, lin_acc_drone_2_lab=True
        )

        print(f"lab_lin_acc: {lab_lin_acc}")

        # measurement in lab frame
        [sensors[i].measure_acceleration(lab_lin_acc[i, 0]) for i in range(3)]
        [sensors[i].measure_acceleration(lab_rot_acc[i - 3, 0]) for i in range(3, 6)]

        # now, everything should be returned in lab coordinates
        lab_pos, lab_lin_vel, drone_angle, drone_angle_vel = get_positions_and_angles(
            sensors
        )

        thrust = controller.translate_input_to_thrust(
            lab_pos, lab_lin_vel, drone_angle, drone_angle_vel, 
            drone_angle, 
            lin_pids, 
            rot_pids)

        print(f"x: {lab_pos[0, 0]}, vx: {lab_lin_vel[0, 0]}, ax: {lab_lin_acc[0, 0]}")
        print(f"y: {lab_pos[1, 0]}, vy: {lab_lin_vel[1, 0]}, ay: {lab_lin_acc[1, 0]}")
        print(f"z: {lab_pos[2, 0]}, vz: {lab_lin_vel[2, 0]}, az: {lab_lin_acc[2, 0]}")

        dh.new_data(
            time=time * delta_t,
            rotation=drone_angle,
            translation=lab_pos,
            thrusters=thrust,
            wind=wind_speed,
            pid=pid_outputs,
        )

    dh.finish()
