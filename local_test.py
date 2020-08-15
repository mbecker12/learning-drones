import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
import numpy as np
from physical_model import rotation_matrix, QuadcopterPhysics, translational_matrix
from PID import PID
from parameters import *
from data_scraper import DataHandler
from sensor import Sensor
from time import sleep
import sys

from verlet import *

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

    initial_thrust = np.array([[0, 0, 0, 0]])
    initial_wind_speed = np.array([[0.0, 0.0, 0.0]])
    initial_angle = np.array([[0, 30, 0]]) * np.pi / 180
    initial_angle_vel = np.array([[0, 0, 0]])
    initial_pos = np.array([[0.0, 0.0, 10.0]])
    initial_vel = np.array([[0.0, 0.0, 0.0]])
    delta_t = 0.01
    timesteps = 5000

    # Initialize Drone Hardware
    dh = DataHandler(
        parentfolder="results", visualize=visualize, n_servers=n_servers, port=port
    )
    sensors = [Sensor(delta_t, initial_pos[0, i], initial_vel[0, i]) for i in range(3)]
    sensors.extend(
        [
            Sensor(delta_t, initial_angle[0, i], initial_angle_vel[0, i])
            for i in range(3)
        ]
    )
    rot_pids = [
        PID(
            kp=1.0,
            ki=0.0,
            kd=0.1,
            timeStep=delta_t,
            setValue=0 * np.pi / 180,
            integralRange=2,
            calculateFlag="rangeExit",
        ),
        PID(
            kp=1.0,
            ki=0.0,
            kd=0.1,
            timeStep=delta_t,
            setValue=0 * np.pi / 180,
            integralRange=2,
            calculateFlag="rangeExit",
        ),
        PID(
            kp=1.0,
            ki=0.0,
            kd=0.1,
            timeStep=delta_t,
            setValue=0,
            integralRange=2,
            calculateFlag="rangeExit",
        ),
    ]

    lin_pids = [
        PID(
            kp=1.0,
            ki=0.0,
            kd=0.9,
            timeStep=delta_t,
            setValue=25,
            integralRange=2,
            calculateFlag="rangeExit",
        ),
        PID(
            kp=1.5,
            ki=0.0,
            kd=0.3,
            timeStep=delta_t,
            setValue=10,
            integralRange=2,
            calculateFlag="rangeExit",
        ),
        PID(
            kp=1.5,
            ki=0.0,
            kd=0.3,
            timeStep=delta_t,
            setValue=10,
            integralRange=2,
            calculateFlag="rangeExit",
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
    )

    # Initialize Values
    previous_wind_speed = initial_wind_speed
    lab_pos = initial_pos.T
    lab_lin_vel = initial_vel.T
    drone_angle = initial_angle.T
    drone_angle_vel = initial_angle_vel.T

    forces, moments = quadcopter.calculate_forces_and_moments(
        thrust=initial_thrust,
        roll=initial_angle[0, 0],
        pitch=initial_angle[0, 1],
        yaw=initial_angle[0, 2],
        wind_speed=previous_wind_speed,
    )

    # force changes
    for time in range(timesteps):
        real_time = time * delta_t
        # sp_roll = 0
        # sp = 00 * np.pi / 180
        # if time < 20:
        #     sp = 30 * np.pi / 180
        # if 20 < time < 120:
        #     sp -= 5 * np.pi / 180
        # if 120 < time < 150:
        #     sp = 0
        # if 150 < time < 250:
        #     sp = -45 * np.pi / 180
        # if 250 < time < 300:
        #     sp = 0
        # if 300 < time < 350:
        #     sp_roll = 45 * np.pi / 180
        #     rot_pids[0].set_setpoint(sp_roll)
        # if 350 < time < 450:
        #     sp_roll = 10 * np.pi / 180
        #     rot_pids[0].set_setpoint(10 * np.pi / 180)
        # if 450 < time < 550:
        #     sp_roll = 0
        #     rot_pids[0].set_setpoint(sp_roll)
        #     sp = -60 * np.pi / 180
        # if 550 < time:
        #     sp = -10 * np.pi / 180
        # rot_pids[1].set_setpoint(sp)
        # dh.new_setpoints(np.array([[sp_roll, sp, 0]]), np.array([[0, 0, 10]]))
        try:
            sleep(0.05)
        except KeyboardInterrupt:
            dh.finish()

        # accelerations in drone frame
        drone_lin_acc, drone_rot_acc = quadcopter.convert_to_acceleration(
            forces, moments
        )

        # handle accelerations:
        lab_lin_acc = np.dot(quadcopter.Rot.T, drone_lin_acc)
        print(f"lab_lin_acc: {lab_lin_acc}")
        Tr = translational_matrix(
            drone_angle[0, 0], drone_angle[1, 0], drone_angle[2, 0]
        )
        lab_rot_acc = np.dot(Tr, drone_rot_acc)

        # measurement in lab frame
        [sensors[i].measure_acceleration(lab_lin_acc[i, 0]) for i in range(3)]
        [sensors[i].measure_acceleration(lab_rot_acc[i - 3, 0]) for i in range(3, 6)]

        # now, everything should be returned in lab coordinates
        pos_x, vel_x = sensors[0].velocity_verlet()
        pos_y, vel_y = sensors[1].velocity_verlet()
        pos_z, vel_z = sensors[2].velocity_verlet()
        roll, vroll = sensors[3].velocity_verlet()
        pitch, vpitch = sensors[4].velocity_verlet()
        yaw, vyaw = sensors[5].velocity_verlet()

        lab_pos = np.array([[pos_x], [pos_y], [pos_z]])
        lab_lin_vel = np.array([[vel_x], [vel_y], [vel_z]])

        drone_angle = np.array([[roll], [pitch], [yaw]])
        drone_angle_vel = np.array([[vroll], [vpitch], [vyaw]])

        # lab_lin_vel = np.dot(quadcopter.Rot.T, drone_lin_vel)
        # lab_rot_vel = np.dot(Tr, drone_rot_vel)
        # lab_pos = lab_pos + verlet_get_delta_x(lab_lin_vel, lab_lin_acc, delta_t)
        # lab_lin_vel = lab_lin_vel + verlet_get_delta_v(lab_lin_acc, lab_lin_acc, delta_t)

        print(f"x: {lab_pos[0, 0]}, vx: {lab_lin_vel[0, 0]}, ax: {lab_lin_acc[0, 0]}")
        print(f"y: {lab_pos[1, 0]}, vy: {lab_lin_vel[1, 0]}, ay: {lab_lin_acc[1, 0]}")
        print(f"z: {lab_pos[2, 0]}, vz: {lab_lin_vel[2, 0]}, az: {lab_lin_acc[2, 0]}")

        rot_inputs = np.array([roll, pitch, yaw])
        rot_outputs = [pid.calculate(rot_inputs[i]) for i, pid in enumerate(rot_pids)]

        lin_inputs = np.array([lab_pos[0, 0], lab_pos[1, 0], lab_pos[2, 0]])
        lin_outputs = [pid.calculate(lin_inputs[i]) for i, pid in enumerate(lin_pids)]
        print("rot_outputs: ", rot_outputs)
        print("lin_outputs: ", lin_outputs)
        delta_x = lin_outputs[0]
        delta_z = lin_outputs[2]
        thrust = quadcopter.control_thrust(
            rot_outputs, roll, pitch, yaw, delta_z, delta_x=delta_x
        )

        print(pos_z)

        dh.new_data(
            time=time + delta_t,
            rotation=np.array([[roll, pitch, yaw]]),
            translation=np.array([[lab_pos[0, 0], lab_pos[1, 0], lab_pos[2, 0]]]),
            thrusters=thrust,
            wind=previous_wind_speed,
            pid=np.array([[*rot_outputs]]),
        )

        forces, moments = quadcopter.calculate_forces_and_moments(
            thrust=thrust,
            roll=drone_angle[0, 0],
            pitch=drone_angle[1, 0],
            yaw=drone_angle[2, 0],
            wind_speed=previous_wind_speed,
        )

    dh.finish()
