import numpy as np
from copy import deepcopy
from drone.drone import Drone


def selection_and_crossover(drones, p_tournament, p_crossover):
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
