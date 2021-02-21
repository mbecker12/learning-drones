import torch
import numpy as np
from drone.quadcopter.physical_model import QuadcopterPhysics
from drone.sensor.sensor import Sensor, get_positions_and_angles
from drone.quadcopter.parameters import *
from datahandling.data_scraper import DataHandler
from evolve.brain import NeuralNetwork, DroneBrain
from drone.onboard_computer import ControlUnit

# from drone.onboard_computer import ControlUnit
delta_t = 0.01

REWARD_CRASHED = -3000
REWARD_TIME_PASSED = -1
REWARD_COIN_DISTANCE = lambda d: -100 * int(d)
REWARD_UNSTABLE = -3
ANGLE_LIMIT = np.pi / 4  # radians
ANGLE_VEL_LIMIT = 20
REWARD_TRAVEL = 50

ZERO_POINT_ZERO = torch.zeros(1, dtype=torch.float32)
ZERO_POINT_ONE = torch.ones(1, dtype=torch.float32) * 0.1


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
        n_runs=8,
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

        self.visualize = visualize
        self.n_servers = n_servers
        self.port = port

        self.dh = DataHandler(
            parentfolder="./evolution",
            visualize=self.visualize,
            n_servers=self.n_servers,
            port=self.port,
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

        # self.controller = NeuralNetwork(
        #     layer_spec={1: 128, 2: 64, 3: 32, 4: 16, 5: 4}, n_inputs=12
        # )

        self.controller = DroneBrain(
            n_inputs=12,
            n_neurons1=128,
            n_neurons2=256,
            n_neurons3=128,
            n_neurons4=64,
            n_outputs=4,
        )

        self.controller.load_state_dict(torch.load("pretrained_velocity_noflush_short"))

        self.distance_to_coin = np.Infinity
        self.reward = 0
        self.coins = 0
        self.flight_time = 0
        self.crashed = False
        self.position = initial_pos
        self.velocity = initial_vel
        self.angle = initial_angle
        self.angle_vel = initial_angle_vel
        self.thrust = initial_thrust
        self.lab_lin_acc = None
        self.lab_rot_acc = None

        # get stats for all <n_runs> runs
        self.distances_to_coin = []
        self.rewards = []
        self.coins_collected = []
        self.flight_times = []
        self.runs_crashed = []

    def enable_visualization(self, visualize, n_servers, port, dir_name="./rewatch"):
        del self.dh
        self.dh = DataHandler(
            parentfolder=dir_name, visualize=visualize, n_servers=n_servers, port=port,
        )

    def translate_input_to_thrust(self, coin_position):
        self.thrust = self.controller.translate_input_to_thrust(
            coin_position - self.position, self.velocity, self.angle, self.angle_vel,
        )

    def status_update(self, time, lin_targets=None):
        """
        Print a formatted string with current velocity, position, etc.
        """
        print(f"time: {time}")

        pos_title = "position:".ljust(25)
        vel_title = "velocity:".rjust(25)
        print(pos_title + vel_title)
        for i in range(3):
            print(
                f"{self.position[i, 0]}".ljust(25) + f"{self.velocity[i, 0]}".rjust(25)
            )
        print()
        angle_title = "angle:".ljust(25)
        angle_vel_title = "angular velocity:".rjust(25)
        print(angle_title + angle_vel_title)
        for i in range(3):
            print(f"{self.angle[i, 0]}".ljust(25) + f"{self.angle_vel[i, 0]}".rjust(25))

        print("\nthrust:")
        thrust_string = ""
        for i in range(4):
            thrust_string += f"{self.thrust[i]}\t"
        print(thrust_string)
        print()

        if lin_targets is not None:
            print(f"x_target: {lin_targets[-3]}")
            print(f"y_target: {lin_targets[-2]}")
            print(f"z_target: {lin_targets[-1]}")
            print()

    def new_data(self, real_time, wind_speed):
        pid_outputs = np.zeros([6, 1])
        self.dh.new_data(
            time=real_time,
            rotation=self.angle,
            translation=self.position,
            thrusters=self.thrust,
            wind=wind_speed,
            pid=pid_outputs,
            lin_acc=self.lab_lin_acc,
            rot_acc=self.lab_rot_acc,
        )

    def measure(self, wind_speed):
        # calculate accelerations
        lab_lin_acc, lab_rot_acc = self.quadcopter.calculate_accelerations(
            self.angle, wind_speed, self.thrust, lin_acc_drone_2_lab=True
        )
        self.lab_lin_acc = lab_lin_acc
        self.lab_rot_acc = lab_rot_acc

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

        if self.position[-1, 0] <= 0 or self.position[-1, 0] > 80:
            # print("Oops! Crashed into the ground!")
            self.reward += REWARD_CRASHED
            self.crashed = True
            return True

        # punishment for overly unstable flight.
        # check both roll and pitch and yaw speed
        # punish each condition separately
        if np.abs(self.angle[0, 0]) > ANGLE_LIMIT:
            self.reward += REWARD_UNSTABLE
        if np.abs(self.angle[1, 0]) > ANGLE_LIMIT:
            self.reward += REWARD_UNSTABLE
        if np.abs(self.angle_vel[2, 0]) > ANGLE_VEL_LIMIT:
            self.reward += REWARD_UNSTABLE
        return False

    def mutate(self, mutation_rate):
        if isinstance(self.controller, torch.nn.Module):
            for layer in self.controller.state_dict():
                shape = self.controller.state_dict()[layer].shape
                uniform_random_vector = torch.rand(shape)
                mutation_mask = uniform_random_vector < mutation_rate
                normal_values = torch.normal(0.0, 0.05, size=shape)
                mutation_deltas = torch.mul(mutation_mask, normal_values)
                self.controller.state_dict()[layer] += mutation_deltas

        else:
            for i, layer in enumerate(self.controller.weights):
                for j, row in enumerate(layer):
                    for k, w in enumerate(row):
                        if np.random.random_sample() < mutation_rate:
                            self.controller.weights[i][j][k] += np.random.normal(
                                0.0, 0.1
                            )

    def reset_reward(self):
        self.reward = 0

    def reset_accumulated_stats(self):
        self.distances_to_coin = []
        self.rewards = []
        self.coins_collected = []
        self.flight_times = []
        self.runs_crashed = []

    def reset(self):
        self.distances_to_coin.append(self.distance_to_coin)
        self.rewards.append(self.reward)
        self.coins_collected.append(self.coins)
        self.flight_times.append(self.flight_time)
        self.runs_crashed.append(self.crashed)

        self.distance_to_coin = np.Infinity
        self.reward = 0
        self.crashed = False
        self.coins = 0
        self.flight_time = 0
        self.position = np.array(initial_pos)
        self.velocity = np.array(initial_vel)
        self.angle = np.array(initial_angle)
        self.angle_vel = np.array(initial_angle_vel)
        self.thrust = np.array(initial_thrust)
        self.lab_rot_acc = None
        self.lab_lin_acc = None

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

        # del self.dh
        self.dh = DataHandler(
            parentfolder="./evolution",
            visualize=self.visualize,
            n_servers=self.n_servers,
            port=self.port,
        )
