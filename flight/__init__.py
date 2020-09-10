import os
from time import sleep
from drone.drone import (
    Drone,
    REWARD_TIME_PASSED,
    REWARD_COIN_DISTANCE,
)
from objects.coins import Coin
from drone.quadcopter.parameters import *


def set_coin_successively(run_idx, radius_sq=50, height=10, total_runs=5):
    phi = run_idx / total_runs * 2 * np.pi
    x = np.sqrt(radius_sq) * np.cos(phi)
    y = np.sqrt(radius_sq) * np.sin(phi)
    z = height

    return np.array([[x], [y], [z]])


def set_coin_delta(run_idx, radius_sq=50, total_runs=5):
    phi = run_idx / total_runs * 2 * np.pi + np.random.normal(0.0, 0.1)
    delta_x = np.sqrt(radius_sq) * np.cos(phi)
    delta_y = np.sqrt(radius_sq) * np.sin(phi)
    delta_z = np.random.normal(0.0, 1.0)

    return np.array([[delta_x], [delta_y], [delta_z]])


def set_coin_position_on_radius(radius_sq=50, height=10):
    phi = np.random.random_sample() * 2 * np.pi
    x = np.sqrt(radius_sq) * np.cos(phi)
    y = np.sqrt(radius_sq) * np.sin(phi)
    z = height

    return np.array([[x], [y], [z]])


def fly(
    drone: Drone,
    timesteps: int = 2500,
    delta_t: float = 0.01,
    visualize=False,
    n_servers=0,
    punish_time=False,
    reward_time=True,
    idx=None,
    run_idx=None,
    total_runs=5,
):

    # print(f"flying in PID: {os.getpid()}")
    # initialize coin
    coin_position = set_coin_successively(
        run_idx, radius_sq=50, height=10, total_runs=total_runs
    )
    # print(run_idx, coin_position)
    coin = Coin(coin_position, 2, 1000)
    # coin = Coin(np.array([[5], [5], [10]]), 2, 1000)

    # initialize drone position
    wind_speed = initial_wind_speed

    # set targets
    rot_targets = np.zeros((3, 1))
    lin_targets = coin.position

    # initialize acceleration; drone should be free falling for now
    lab_lin_acc, lab_rot_acc = drone.quadcopter.calculate_accelerations(
        drone.angle, wind_speed, drone.thrust, lin_acc_drone_2_lab=True
    )

    for time in range(timesteps):
        real_time = time * delta_t
        if punish_time:
            drone.reward -= REWARD_TIME_PASSED
        if reward_time:
            drone.reward += REWARD_TIME_PASSED
        drone.flight_time += 1

        collected_coin = coin.is_in_vicinity(drone.position)
        if collected_coin:
            drone.reward += coin.value
            drone.coins += 1

            # coin = Coin(coin.position + np.array([[5.0], [2.5], [0.0]]), 2, value=2000)
            coin_pos_delta = set_coin_delta(
                run_idx, radius_sq=50, total_runs=total_runs
            )
            coin = Coin(coin.position + coin_pos_delta, 2, value=2000)
            # coin = Coin(
            #     coin.position + (10 * np.random.random_sample(coin.position.shape)),
            #     1,
            #     value=1000
            #     )
            lin_targets = coin.position
            if visualize:
                drone.dh.new_setpoints(rot_targets, lin_targets)
                drone.dh.new_setpoints(rot_targets, lin_targets)
                drone.dh.new_setpoints(rot_targets, lin_targets)

        changed_setpoint = False
        if time % 10 == 0:
            changed_setpoint = True

        if visualize and changed_setpoint:
            drone.dh.new_setpoints(rot_targets, lin_targets)

        try:
            if not visualize and n_servers < 2:
                pass
            else:
                sleep(0.1)
        except KeyboardInterrupt:
            drone.dh.finish()

        is_crashed = drone.measure(wind_speed=wind_speed)
        if is_crashed:
            if visualize:
                drone.dh.finish(drone.reward, quit_program=False)
            drone.reward += REWARD_COIN_DISTANCE(
                coin.distance_drone_to_coin(drone.position)
            )
            drone.distance_to_coin = coin.distance_drone_to_coin(drone.position)
            return drone.reward, real_time, idx

        drone.translate_input_to_thrust(coin.position)

        if visualize:
            drone.status_update(real_time, lin_targets=coin.position)
            drone.new_data(real_time, wind_speed)

    if visualize:
        drone.dh.finish(drone.reward, quit_program=False)
    drone.reward += REWARD_COIN_DISTANCE(coin.distance_drone_to_coin(drone.position))
    drone.distance_to_coin = coin.distance_drone_to_coin(drone.position)
    return drone.reward, real_time, idx
