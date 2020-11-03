import numpy as np


def set_coin_successively(run_idx, radius_sq=50, height=10, total_runs=5):
    phi = run_idx / total_runs * 2 * np.pi
    x = np.sqrt(radius_sq) * np.cos(phi)
    y = np.sqrt(radius_sq) * np.sin(phi)
    z = height

    return np.array([[x], [y], [z]])


def set_coin_delta(
    run_idx,
    radius_sq=50,
    total_runs=5,
    height_std=1.0,
):
    # TODO: find a good way to randomize new coin positions
    # without messing up elitism
    phi = run_idx / total_runs * 2 * np.pi + np.random.normal(0.0, 0.02)
    delta_x = np.sqrt(radius_sq) * np.cos(phi)
    delta_y = np.sqrt(radius_sq) * np.sin(phi)
    delta_z = np.random.normal(0.0, 0.2)
    # delta_z = 0

    return np.array([[delta_x], [delta_y], [delta_z]])


def set_coin_position_on_radius(radius_sq=50, height=10):
    phi = np.random.random_sample() * 2 * np.pi
    x = np.sqrt(radius_sq) * np.cos(phi)
    y = np.sqrt(radius_sq) * np.sin(phi)
    z = height

    return np.array([[x], [y], [z]])


def create_new_random_position(prev_x, prev_y, prev_z):
    new_x = -999
    while not (-100 <= new_x <= 100):
        delta_x = np.random.uniform(-20, 20)
        new_x = prev_x + delta_x

    new_y = -999
    while not (-100 <= new_y <= 100):
        delta_y = np.random.uniform(-20, 20)
        new_y = prev_y + delta_y

    new_z = -999
    while not (0 < new_z <= 30):
        delta_z = np.random.uniform(-5, 5)
        new_z = prev_z + delta_z

    # return np.array([[new_x], [new_y], [new_z]])
    return [new_x, new_y, new_z]


def create_choreo(sequence_length, verbose=False):
    if verbose:
        print("Initialize Position")
    # initial_pos = np.array([[0],[0],[10]])
    initial_pos = [0, 0, 10]

    position_sequence = [None] * (sequence_length + 1)
    position_sequence[0] = initial_pos

    for i in range(1, sequence_length + 1):
        pos = create_new_random_position(
            position_sequence[i - 1][0],
            position_sequence[i - 1][1],
            position_sequence[i - 1][2],
        )

        position_sequence[i] = pos

    position_sequence.pop(0)
    if verbose:
        print(f"Final sequence:")
    if verbose:
        for i in range(sequence_length):
            print(
                f"x: {position_sequence[i][0, 0]:3.4f}, \ty: {position_sequence[i][1, 0]:3.4f}, \tz: {position_sequence[i][2, 0]:3.4f}"
            )

    assert len(position_sequence) == sequence_length
    return position_sequence


def set_predetermined_coin(run_idx, coin_idx):
    pass


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    plot = False

    if plot:
        plt.ion()
    modes = ["train", "val"]

    n_runs = {"train": 20, "val": 10}
    n_coins = 20
    sequences = {}
    for mode in modes:
        sequences[mode] = {}

        for run in range(n_runs[mode]):
            sequences[mode][run] = {}
            position_sequence = create_choreo(n_coins)

            for coin_idx in range(n_coins):
                sequences[mode][run][coin_idx] = {
                    "x": position_sequence[coin_idx][0],
                    "y": position_sequence[coin_idx][1],
                    "z": position_sequence[coin_idx][2],
                    "value": 2000,
                    "radius": 2,
                }
                # print(np.array(position_sequence[:coin_idx+1])[:, 0])
                if plot:
                    plt.scatter(
                        np.array(position_sequence[: coin_idx + 1])[:, 0],
                        np.array(position_sequence[: coin_idx + 1])[:, 1],
                    )
                    plt.xlim(-100, 100)
                    plt.ylim(-100, 100)
                    plt.draw()
                    plt.pause(0.5)
            if plot:
                plt.clf()
                plt.cla()
    import json
    import yaml

    with open("choreo.json", "w") as choreo_json:
        json.dump(sequences, choreo_json)
    with open("choreo.yaml", "w") as choreo_yaml:
        yaml.dump(sequences, choreo_yaml)
    # print(sequences)
