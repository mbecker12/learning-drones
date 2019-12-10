import numpy as np
from physical_model import rotation_matrix, QuadcopterPhysics
from PID import PID
from parameters import *
from data_scraper import DataHandler
from sensor import Sensor
from time import sleep

thrust_type = 'yaw'

if __name__ == "__main__":
    # initialize drone, choose starting position
    initial_thrust = np.array([[1, 1, 1, 1]])
    initial_roll = 0.0
    initial_pitch = 0.0
    initial_yaw = 0.0
    initial_vroll = 0.0
    initial_vpitch = 0.0
    initial_vyaw = 0.0
    initial_wind_speed = 0.0
    initial_x = 10.0
    initial_y = 10.0
    initial_z = 10.0
    initial_vx = 0.0
    initial_vy = 0.0
    initial_vz = 0.0
    delta_t = 0.01
    
    thrusts = np.array([
        [[1, 1, 1, 1]],
        [[1, 1, 1, 1]],
        [[1, 1, 1, 1]],
        [[1, 0, 0, 1]],
        [[1, 0, 0, 1]],
        [[1, 0, 0, 1]],
        [[1, 0, 1, 0]],
        [[1, 0, 1, 0]],
        [[1, 0, 1, 0]],
        [[1, 1, 0, 0]],
        [[1, 1, 0, 0]],
        [[1, 1, 0, 0]],
    ])

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
    dh = DataHandler(parentfolder="results", visualize=True, n_servers=1)
    sensors = [Sensor(delta_t) for _ in range(6)]
    pids = [
        PID(kp=1, ki=1, kd=1, timeStep=delta_t, setValue=0, integralRange=np.zeros(100), calculateFlag="rangeExit"),
        PID(kp=1, ki=1, kd=1, timeStep=delta_t, setValue=0, integralRange=np.zeros(100), calculateFlag="rangeExit"),
        PID(kp=1, ki=1, kd=1, timeStep=delta_t, setValue=0, integralRange=np.zeros(100), calculateFlag="rangeExit")
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
    j = 5
    for time in range(timesteps):
        try:
            sleep(0.1)
        except KeyboardInterrupt:
            dh.finish()
        # NB Do we input the correct thrust
        forces, moments = quadcopter.calculate_forces_and_moments(
            thrust=previous_thrust,
            roll=previous_roll,
            pitch=previous_pitch,
            yaw=previous_yaw,
            wind_speed=previous_wind_speed)

        lin_acc, rot_acc = quadcopter.convert_to_acceleration(forces, moments)

        [sensors[i].measure_acceleration(lin_acc[i, 0]) for i in range(3)]
        [sensors[i].measure_acceleration(rot_acc[i-3, 0]) for i in range(3, 6)]

        pos_x, vel_x = sensors[0].velocity_verlet(previous_x, previous_vx)
        pos_y, vel_y = sensors[1].velocity_verlet(previous_y, previous_vy)
        pos_z, vel_z = sensors[2].velocity_verlet(previous_z, previous_vz)
        roll, vroll = sensors[3].velocity_verlet(previous_roll, previous_vroll)
        pitch, vpitch = sensors[4].velocity_verlet(previous_pitch, previous_vpitch)
        yaw, vyaw = sensors[5].velocity_verlet(previous_yaw, previous_vyaw)

        inputs = np.zeros(len(pids))
        outputs = [pid.calculate(inputs[i]) for i, pid in enumerate(pids)]

        delta_z = 0.0
        # replace thruster values by hard coded values
        # thrust = quadcopter.controll_thrust(outputs, roll, pitch, yaw, delta_z)

        thrust = thrust_dict[thrust_type]
        # thrust *= np.sin(time * 2 * np.pi)
        print(thrust)
        if j == 20:
            j = 0
        if 8 < j < 20:
            thrust = 1 - thrust
        j += 1


        dh.new_data(
            time=time + delta_t,
            rotation=np.array([[roll, pitch, yaw]]),
            translation=np.array([[pos_x, pos_y, pos_z]]),
            thrusters=previous_thrust,
            wind=previous_wind_speed)

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

    dh.finish()
