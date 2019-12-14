import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
import numpy as np
from physical_model import rotation_matrix, QuadcopterPhysics
from PID import PID
from parameters import *
from data_scraper import DataHandler
from sensor import Sensor
from time import sleep
import sys

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
    initial_thrust = np.array([[0, 0, 0, 0]])
    # initialize randomly
    initial_roll = np.random.rand() * 2 - 1
    initial_pitch = np.random.rand() * 2 - 1
    initial_yaw = np.random.rand() * 2 - 1
    initial_vroll = np.random.rand() * 2 - 1
    initial_vpitch = np.random.rand() * 2 - 1
    initial_vyaw = np.random.rand() * 2 - 1
    initial_wind_speed = np.array([[0.0, 0.0, 0.0]])
    # initialize deterministically
    initial_roll = 30 * np.pi / 180
    initial_pitch = 00 * np.pi / 180
    initial_yaw = 0 * np.pi / 180
    initial_vroll = 0.0
    initial_vpitch = 0.0
    initial_vyaw = 0.0
    initial_wind_speed = np.array([[0.0, 0.0, 0.0]])

    initial_x = 10.0
    initial_y = 10.0
    initial_z = 10.0
    initial_vx = 0.0
    initial_vy = 0.0
    initial_vz = 0.0
    delta_t = 0.01

    roll_thrust = np.array([[1, 0, 0, 1]], dtype=np.float32)
    pitch_thrust = np.array([[1, 1, 0, 0]], dtype=np.float32)
    yaw_thrust = np.array([[1, 0, 1, 0]], dtype=np.float32)
    thrust_dict = {
        'roll': roll_thrust,
        'pitch': pitch_thrust,
        'yaw': yaw_thrust
    }

    timesteps = 1000
    
    # Initialize Drone Hardware
    dh = DataHandler(parentfolder="results", visualize=visualize, n_servers=n_servers, port=port)
    sensors = [Sensor(delta_t) for _ in range(6)]
    rot_pids = [
        PID(kp=1, ki=0.0, kd=0.1, timeStep=delta_t, setValue=30 * np.pi / 180, integralRange=2, calculateFlag="rangeExit"),
        PID(kp=1, ki=0.0, kd=0.1, timeStep=delta_t, setValue=00 * np.pi / 180, integralRange=2, calculateFlag="rangeExit"),
        PID(kp=1, ki=0.0, kd=0.1, timeStep=delta_t, setValue=0, integralRange=2, calculateFlag="rangeExit")
    ]

    lin_pids = [
        PID(kp=1.5, ki=0.0, kd=0.3, timeStep=delta_t, setValue=10, integralRange=2, calculateFlag="rangeExit")
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
        y_payload=y_payload
    )

    # Initialize Values
    previous_wind_speed = initial_wind_speed
    previous_x = initial_x
    previous_y = initial_y
    previous_z = initial_z
    previous_vx = initial_vx
    previous_vy = initial_vy
    previous_vz = initial_vz

    previous_roll = initial_roll
    previous_pitch = initial_pitch
    previous_yaw = initial_yaw
    previous_vroll = initial_vroll
    previous_vpitch = initial_vpitch
    previous_vyaw = initial_vyaw
    previous_thrust = initial_thrust

    forces, moments = quadcopter.calculate_forces_and_moments(
            thrust=previous_thrust,
            roll=previous_roll,
            pitch=previous_pitch,
            yaw=previous_yaw,
            wind_speed=previous_wind_speed)

    for time in range(timesteps):
        real_time = time * delta_t
        try:
            sleep(0.2)
        except KeyboardInterrupt:
            dh.finish()

        # accelerations in drone frame
        lin_acc, rot_acc = quadcopter.convert_to_acceleration(forces, moments)

        # handle accelerations:
        lin_acc_lab = np.dot(quadcopter.Rot, lin_acc)
        print(f"lin_acc_lab: {lin_acc_lab}")

        [sensors[i].measure_acceleration(lin_acc[i, 0]) for i in range(3)]
        [sensors[i].measure_acceleration(rot_acc[i-3, 0]) for i in range(3, 6)]

        pos_x, vel_x = sensors[0].velocity_verlet(previous_x, previous_vx)
        pos_y, vel_y = sensors[1].velocity_verlet(previous_y, previous_vy)
        pos_z, vel_z = sensors[2].velocity_verlet(previous_z, previous_vz)
        roll, vroll = sensors[3].velocity_verlet(previous_roll, previous_vroll)
        pitch, vpitch = sensors[4].velocity_verlet(previous_pitch, previous_vpitch)
        yaw, vyaw = sensors[5].velocity_verlet(previous_yaw, previous_vyaw)

        # print(f"x: {pos_x}, vx: {vel_x}, ax: {sensors[0].return_acceleration()}")
        # print(f"y: {pos_y}, vy: {vel_y}, ay: {sensors[1].return_acceleration()}")
        # print(f"z: {pos_z}, vz: {vel_z}, az: {sensors[2].return_acceleration()}")

        # inputs = np.zeros(len(pids))
        rot_inputs = np.array([roll, pitch, yaw])
        rot_outputs = [pid.calculate(rot_inputs[i]) for i, pid in enumerate(rot_pids)]

        lin_inputs = np.array([pos_z])
        lin_outputs = [pid.calculate(lin_inputs[i]) for i, pid in enumerate(lin_pids)]
        print("rot_outputs: ", rot_outputs)
        print("lin_outputs: ", lin_outputs)
        delta_z = lin_outputs[0]
        # delta_z = 0.0
      
        thrust = quadcopter.control_thrust(rot_outputs, roll, pitch, yaw, delta_z)
        thrust = np.array([[0., 0., 0., 0.]])
        print(pos_z)
        

        # Update Values
        previous_x = pos_x
        previous_y = pos_y
        previous_z = pos_z
        previous_vx = vel_x
        previous_vy = vel_y
        previous_vz = vel_z
        previous_roll = roll
        previous_pitch = pitch
        previous_yaw = yaw
        previous_vroll = vroll
        previous_vpitch = vpitch
        previous_vyaw = vyaw
        previous_thrust = thrust

        dh.new_data(
            time=time + delta_t,
            rotation=np.array([[roll, pitch, yaw]]),
            translation=np.array([[pos_x, pos_y, pos_z]]),
            thrusters=previous_thrust,
            wind=previous_wind_speed, pid=np.array([[*rot_outputs]]))

        forces, moments = quadcopter.calculate_forces_and_moments(
            thrust=previous_thrust,
            roll=previous_roll,
            pitch=previous_pitch,
            yaw=previous_yaw,
            wind_speed=previous_wind_speed)

    dh.finish()
