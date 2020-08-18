import numpy as np
from drone.quadcopter.physical_model import QuadcopterPhysics
from drone.sensor.sensor import Sensor, get_positions_and_angles
from drone.quadcopter.parameters import *
from datahandling.data_scraper import DataHandler
from evolve.brain import NeuralNetwork
from drone.onboard_computer import ControlUnit

# from drone.onboard_computer import ControlUnit
delta_t = 0.01


class Drone(ControlUnit):
    def __init__(
        self,
        port: int = 65432,
        visualize=True,
        n_servers=1,
        initial_pos=np.array([[0.0], [0.0], [10.0]]),
        initial_vel=np.array([[0.0], [0.0], [0.0]]),
        initial_angle=np.array([[0.0], [0.0], [0.0]]),
        initial_angle_vel=np.array([[0.0], [0.0], [0.0]]),
    ):

        self.quadcopter = QuadcopterPhysics(
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

        self.dh = DataHandler(
            parentfolder="./evolution",
            visualize=visualize,
            n_servers=n_servers,
            port=port,
        )

        self.sensors = [
            Sensor(delta_t, initial_pos[i, 0], initial_vel[i, 0]) for i in range(3)
        ]
        self.sensors.extend(
            [
                Sensor(delta_t, initial_angle[i, 0], initial_angle_vel[i, 0])
                for i in range(3)
            ]
        )

        self.controller = NeuralNetwork(layer_spec={1: 8, 2: 10, 3: 4})

        self.reward = 0
        self.position = initial_pos
        self.velocity = initial_vel
        self.angle = initial_angle
        self.angle_vel = initial_angle_vel
        self.thrust = initial_thrust

    def translate_input_to_thrust(self, coin_position):
        self.thrust = self.controller.translate_input_to_thrust(
            self.position, self.velocity, self.angle, self.angle_vel, coin_position,
        )

    def status_update(self, time, lin_targets=None):
        print(f"time: {time}")
        print(f"position: \n{self.position}")
        print(f"velocity: \n{self.velocity}")
        print(f"angle: \n{self.angle}")
        print(f"angle velocity: \n{self.angle_vel}")
        print(f"thrust: \n{self.thrust}")
        if lin_targets is not None:
            print(f"x_target: {lin_targets[-3]}")
            print(f"y_target: {lin_targets[-2]}")
            print(f"z_target: {lin_targets[-1]}")

    def new_data(self, real_time, wind_speed):
        pid_outputs = np.zeros([6, 1])
        self.dh.new_data(
            time=real_time,
            rotation=self.angle,
            translation=self.position,
            thrusters=self.thrust,
            wind=wind_speed,
            pid=pid_outputs,
        )

    def measure(self, wind_speed):
        # calculate accelerations
        lab_lin_acc, lab_rot_acc = self.quadcopter.calculate_accelerations(
            self.angle, wind_speed, self.thrust, lin_acc_drone_2_lab=True
        )

        # measurement in lab frame
        [self.sensors[i].measure_acceleration(lab_lin_acc[i, 0]) for i in range(3)]
        [
            self.sensors[i].measure_acceleration(lab_rot_acc[i - 3, 0])
            for i in range(3, 6)
        ]

        # now, everything should be returned in lab coordinates
        (
            self.position,
            self.velocity,
            self.angle,
            self.angle_vel,
        ) = get_positions_and_angles(self.sensors)

        if self.position[-1] <= 0:
            # print("Oops! Crashed into the ground!")
            self.reward -= 10000
            return True
        return False

    def mutate(self, mutation_rate):
        for i, layer in enumerate(self.controller.weights):
            for j, row in enumerate(layer):
                for k, w in enumerate(row):
                    if np.random.random_sample() < mutation_rate:
                        self.controller.weights[i][j][k] += np.random.normal(0.0, 0.3)

    def reset_reward(self):
        self.reward = 0

    def reset(self):
        self.reward = 0
        self.position = np.array(initial_pos)
        self.velocity = np.array(initial_vel)
        self.angle = np.array(initial_angle)
        self.angle_vel = np.array(initial_angle_vel)
        self.thrust = np.array(initial_thrust)

        self.sensors = [
            Sensor(delta_t, np.array(initial_pos)[i, 0], np.array(initial_vel)[i, 0])
            for i in range(3)
        ]
        self.sensors.extend(
            [
                Sensor(
                    delta_t,
                    np.array(initial_angle)[i, 0],
                    np.array(initial_angle_vel)[i, 0],
                )
                for i in range(3)
            ]
        )

        self.quadcopter = QuadcopterPhysics(
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
