import numpy as np
from copy import deepcopy
import sys
from time import sleep, time

sys.path.append("/home/marvin/Projects/Drones")
from drone.drone import Drone
from flight import fly
import jsonpickle
import json

n_generations = 100
n_drones = 10
mutation_rate = 0.1
p_tournament = 0.8
p_crossover = 0.05


def selection(drones, p_tournament):
    n = len(drones)
    new_population = deepcopy(drones)
    for i in range(0, n, 2):
        idx1 = np.random.randint(0, n)
        idx2 = np.random.randint(0, n)
        idx3 = np.random.randint(0, n)
        idx4 = np.random.randint(0, n)
        selected_idx1 = tournament_select(drones, idx1, idx2, p_tournament)
        selected_idx2 = tournament_select(drones, idx3, idx4, p_tournament)

        new_population[i] = deepcopy(drones[selected_idx1])
        new_population[i + 1] = deepcopy(drones[selected_idx2])

        new_weights_1, new_weights_2 = crossover(
            new_population[i].controller.weights,
            new_population[i + 1].controller.weights,
            p_crossover,
        )
        new_population[i].controller.weights = new_weights_1
        new_population[i + 1].controller.weights = new_weights_2

    return new_population


def tournament_select(drones, idx1, idx2, p_tournament):
    r = np.random.random_sample()
    score1 = drones[idx1].reward
    score2 = drones[idx2].reward
    if r < p_tournament:
        if score1 > score2:
            selected_idx = idx1
        else:
            selected_idx = idx2
    else:
        if score1 > score2:
            selected_idx = idx2
        else:
            selected_idx = idx1

    return selected_idx


def crossover(weights1, weights2, p_crossover):
    new_weights_1 = []
    new_weights_2 = []
    for i, layer in enumerate(weights1):
        r = np.random.random_sample()
        if r < p_crossover:
            lead_dimension = layer.shape[0]
            n_genes = lead_dimension * layer.shape[1]
            crossover_point = np.random.randint(0, n_genes)

            flat_layer_1 = layer.flatten()
            flat_layer_2 = weights2[i].flatten()
            new_chromosome_1 = np.empty(flat_layer_1.shape)
            new_chromosome_2 = np.empty(flat_layer_2.shape)

            new_chromosome_1[:crossover_point] = flat_layer_1[:crossover_point]
            new_chromosome_1[crossover_point:] = flat_layer_2[crossover_point:]

            new_chromosome_2[:crossover_point] = flat_layer_2[:crossover_point]
            new_chromosome_2[crossover_point:] = flat_layer_1[crossover_point:]

            new_chromosome_1 = new_chromosome_1.reshape(lead_dimension, -1)
            new_chromosome_2 = new_chromosome_2.reshape(lead_dimension, -1)

            new_weights_1.append(new_chromosome_1)
            new_weights_2.append(new_chromosome_2)
        else:
            new_weights_1.append(weights1[i])
            new_weights_2.append(weights2[i])

    return new_weights_1, new_weights_2


def get_best_of_generation(scores):
    best_score = -np.Infinity
    best_index = -1

    for i, score in enumerate(scores):
        if score > best_score:
            best_score = score
            best_index = i

    return best_index, best_score


def compare_weights(weights1, weights2):
    if isinstance(weights1, Drone):
        weights1 = weights1.controller.weights
    if isinstance(weights2, Drone):
        weights2 = weights2.controller.weights

    for i, layer in enumerate(weights1):
        for j, row in enumerate(layer):
            for k, w in enumerate(row):
                assert (
                    w == weights2[i][j][k]
                ), f"i: {i}, j: {j}, k: {k}, w1: {w}, w2: {weights2[i][j][k]}"


if __name__ == "__main__":
    # drone = Drone(visualize=False, n_servers=0)
    # sleep(2)
    # fly(drone)
    # exit()

    execution_time = int(time())
    drones = [Drone(visualize=False, n_servers=0) for _ in range(n_drones)]
    scores = [0] * n_drones
    times = [0] * n_drones

    global_best_score = -np.Infinity
    global_best_drone = None

    for gen in range(n_generations):
        # TODO: find a way to do this in separate threads
        for i, drone in enumerate(drones):
            score, flight_time = fly(drone)
            scores[i] = score
            times[i] = flight_time

        best_index, best_score = get_best_of_generation(scores)

        if best_score > global_best_score:
            print(f"Found new global best! At index {best_index}")
            global_best_score = best_score
            global_best_drone = deepcopy(drones[best_index])

            best_drone_json = jsonpickle.encode(global_best_drone)
            with open(f"best_drone_{execution_time}.json", "w") as jsonfile:
                json.dump(best_drone_json, jsonfile)

        new_population = selection(drones, p_tournament)

        for i, drone in enumerate(new_population):
            drone.reset()
            drone.mutate(mutation_rate)

        drones = new_population
        drones[0] = global_best_drone
        drones[0].reset()

        print(
            f"Generation {gen}: Best Score: {best_score}, Global Best: {global_best_score}"
        )
        print(f"Scores: {scores}")
        print(f"Times: {times}")
        print()
