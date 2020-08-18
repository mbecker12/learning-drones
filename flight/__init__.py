from time import sleep
from drone.drone import Drone
from objects.coins import Coin
from drone.quadcopter.parameters import *


def fly(
    drone: Drone,
    timesteps: int = 1000,
    delta_t: float = 0.01,
    visualize=False,
    n_servers=0,
    punish_time=False,
    reward_time=True,
):
    # initialize coin
    coin = Coin(np.array([[5], [5], [10]]), 1, 1000)

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
            drone.reward -= 2
        if reward_time:
            drone.reward += 2

        collected_coin = coin.is_in_vicinity(drone.position)
        if collected_coin:
            drone.reward += coin.value

            coin = Coin(coin.position + np.array([[5.0], [2.5], [0.0]]), 1, value=1000)
            # coin = Coin(
            #     coin.position + (10 * np.random.random_sample(coin.position.shape)),
            #     1,
            #     value=1000
            #     )
            lin_targets = coin.position
            # drone.dh.new_setpoints(rot_targets, lin_targets)
            # drone.dh.new_setpoints(rot_targets, lin_targets)
            # drone.dh.new_setpoints(rot_targets, lin_targets)

        changed_setpoint = False
        if time % 10 == 0:
            changed_setpoint = True

        # if changed_setpoint: drone.dh.new_setpoints(rot_targets, lin_targets)

        try:
            if not visualize and n_servers < 2:
                pass
            else:
                sleep(0.1)
        except KeyboardInterrupt:
            drone.dh.finish()

        is_crashed = drone.measure(wind_speed=wind_speed)
        if is_crashed:
            # drone.dh.finish(drone.reward, quit_program=False)
            drone.reward -= 10 * int(coin.distance_drone_to_coin(drone.position))
            return drone.reward, real_time

        drone.translate_input_to_thrust(coin.position)

        # drone.status_update(real_time)
        # drone.new_data(real_time, wind_speed)

    # drone.dh.finish(drone.reward, quit_program=False)
    drone.reward -= 10 * int(coin.distance_drone_to_coin(drone.position))
    return drone.reward, real_time
