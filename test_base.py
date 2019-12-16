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
import sys

from verlet import *

thrust_type = 'pitch'

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
    initial_angle = np.array([[0, 30, 0]]).T * np.pi / 180
    initial_angle_vel = np.array([[0, 0, 0]]).T
    initial_pos = np.array([[0.0, 0.0, 10.0]]).T
    initial_vel = np.array([[0.0, 0.0, 0.0]]).T
    delta_t = 0.01
    timesteps = 5000

    x_target = 25
    y_target = 0
    z_target = 10
    # Initialize Drone Hardware
    dh = DataHandler(parentfolder="results", visualize=visualize, n_servers=n_servers, port=port)
    sensors = [Sensor(delta_t, initial_pos[i, 0], initial_vel[i, 0]) for i in range(3)]
    sensors.extend([Sensor(delta_t, initial_angle[i, 0], initial_angle_vel[i, 0]) for i in range(3)])

    rot_pids = [
        PID(kp=1., ki=0.0, kd=0.1, timeStep=delta_t, setValue=0 * np.pi / 180, integralRange=2, calculateFlag="rangeExit"),
        PID(kp=1., ki=0.0, kd=0.1, timeStep=delta_t, setValue=0 * np.pi / 180, integralRange=2, calculateFlag="rangeExit"),
        PID(kp=1., ki=0.0, kd=0.1, timeStep=delta_t, setValue=0, integralRange=2, calculateFlag="rangeExit")
    ]

    lin_pids = [
        PID(kp=1.0, ki=0.0, kd=0.9, timeStep=delta_t, setValue=x_target, integralRange=2, calculateFlag="rangeExit"),
        PID(kp=1.5, ki=0.0, kd=0.3, timeStep=delta_t, setValue=y_target, integralRange=2, calculateFlag="rangeExit"),
        PID(kp=1.5, ki=0.0, kd=0.3, timeStep=delta_t, setValue=z_target, integralRange=2, calculateFlag="rangeExit")
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
        I_z=I_z
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
            wind_speed=wind_speed)

    # force changes
    for time in range(timesteps):
        real_time = time * delta_t
        sp_roll = 0
        sp = 00 * np.pi / 180
        if time < 5:
            changed_setpoint = True
        else:
            changed_setpoint = False

        if changed_setpoint:
            dh.new_setpoints(np.array([[sp_roll, sp, 0]]).T, np.array([[x_target, y_target, z_target]]).T)
            
        try:
            sleep(0.05)
        except KeyboardInterrupt:
            dh.finish()

        # calculate accelerations
        lab_lin_acc, lab_rot_acc = quadcopter.calculate_accelerations(
            drone_angle, wind_speed, thrust, lin_acc_drone_2_lab=True)
        
        print(f"lab_lin_acc: {lab_lin_acc}")
        # Tr = translational_matrix(drone_angle[0, 0], drone_angle[1, 0], drone_angle[2, 0])
        # lab_rot_acc = np.dot(Tr, drone_rot_acc)

        # measurement in lab frame
        [sensors[i].measure_acceleration(lab_lin_acc[i, 0]) for i in range(3)]
        [sensors[i].measure_acceleration(lab_rot_acc[i-3, 0]) for i in range(3, 6)]

        # now, everything should be returned in lab coordinates
        lab_pos, lab_lin_vel, drone_angle, drone_angle_vel = get_positions_and_angles(sensors)

        print(f"x: {lab_pos[0, 0]}, vx: {lab_lin_vel[0, 0]}, ax: {lab_lin_acc[0, 0]}")
        print(f"y: {lab_pos[1, 0]}, vy: {lab_lin_vel[1, 0]}, ay: {lab_lin_acc[1, 0]}")
        print(f"z: {lab_pos[2, 0]}, vz: {lab_lin_vel[2, 0]}, az: {lab_lin_acc[2, 0]}")

        rot_inputs = drone_angle
        rot_outputs = [pid.calculate(rot_inputs[i, 0]) for i, pid in enumerate(rot_pids)]
        lin_inputs = lab_pos
        lin_outputs = [pid.calculate(lin_inputs[i, 0]) for i, pid in enumerate(lin_pids)]
        print("rot_outputs: ", rot_outputs)
        print("lin_outputs: ", lin_outputs)
        delta_x = lin_outputs[0]
        delta_z = lin_outputs[2]

        # thrust = quadcopter.calculate_motor_thrust(np.concatenate([[rot_outputs], [lin_outputs]], axis=1).T, drone_angle)
        thrust = quadcopter.control_thrust(
            rot_outputs, drone_angle[0, 0], drone_angle[1, 0], drone_angle[2, 0], delta_z, delta_x=delta_x)

        dh.new_data(
            time=time + delta_t,
            rotation=drone_angle,
            translation=lab_pos,
            thrusters=thrust,
            wind=wind_speed,
            pid=np.array([[*rot_outputs, *lin_outputs]]).T)

        forces, moments = quadcopter.calculate_forces_and_moments(
            thrust=thrust,
            roll=drone_angle[0, 0],
            pitch=drone_angle[1, 0],
            yaw=drone_angle[2, 0],
            wind_speed=wind_speed)


    dh.finish()
