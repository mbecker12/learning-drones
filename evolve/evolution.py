import torch
import numpy as np
from copy import deepcopy
import sys
from time import sleep, time
import traceback
import os

# possibly bad style, but it works...
sys.path.append(os.getcwd())

from drone.drone import Drone
from evolve.util import (
    selection_and_crossover,
    tournament_select,
    get_best_of_generation,
)
from flight import fly
import jsonpickle
import json
from multiprocessing.pool import ThreadPool, Pool
from multiprocessing import Manager
import logging

# "evolution_log",
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("evolution")

# from multiprocessing import shared_memory

n_generations = 500
n_drones = 40
initial_mutation_rate = 0.05
mutation_rate0 = 0.05
mutation_rate_decay = 0.999
p_tournament = 0.8
p_crossover = 0.3
n_executions = 5


def thread_prompt(scores, times, result):
    score, flight_time, idx = result
    scores[idx] = score
    times[idx] = flight_time


def thread_error_prompt(err):
    traceback.print_exception(type(err), err, err.__traceback__)
    raise type(err)(err)


# TODO:
# find a way to produce random sequences of coin positions
# and still measure best overall performance
# in order for elitism to work properly

if __name__ == "__main__":
    init_time = time()
    execution_time = int(time())
    logger.info(f"Starting neuroevolution with execution time {execution_time}")
    drones = [Drone(visualize=False, n_servers=0) for _ in range(n_drones)]
    scores = [0] * n_drones
    times = [0] * n_drones

    for i, drone in enumerate(drones):
        # mutate all pretrained drones once, otherwise they all score the same
        drone.reset()
        drone.mutate(initial_mutation_rate)

    global_best_score = -np.Infinity
    global_best_drone = None

    thread_pool = ThreadPool(1)

    logger.debug(f"Initialization time: {time() - init_time}")
    start_time = time()
    for gen in range(n_generations):

        mutation_rate = mutation_rate0 * mutation_rate_decay ** gen
        generation_time = time() - start_time
        logger.info(f"Time elapsed for last generation: {generation_time}")
        start_time = time()
        # TODO: find a way to do this in separate threads

        # TODO: threads won't be the answer; need to use processes
        # TODO: For that, we need to find a way to share complex/nested objects between processes
        # maybe this will help:
        # https://docs.python.org/3/library/multiprocessing.html#proxy-objects
        # https://www.geeksforgeeks.org/multiprocessing-python-set-2/
        fly_time = time()
        for i, drone in enumerate(drones):
            avg_score = 0
            avg_flight_time = 0
            for j in range(n_executions):
                score, flight_time, idx = fly(
                    drone, idx=i, run_idx=j, total_runs=n_executions, timesteps=2000
                )
                avg_score += score
                avg_flight_time += flight_time
                drone.reset()

            scores[idx] = avg_score / n_executions
            times[idx] = avg_flight_time / n_executions
        logger.debug(f"Fly time: {time() - fly_time}")
        best_index = np.argmax(scores)
        best_score = scores[best_index]

        save_time = time()
        if best_score > global_best_score:
            logger.info(f"Found new global best! At index {best_index}")
            global_best_score = best_score
            global_best_drone = deepcopy(drones[best_index])

            best_drone_json = jsonpickle.encode(global_best_drone)
            with open(f"evolution/best_drone_{execution_time}.json", "w") as jsonfile:
                json.dump(best_drone_json, jsonfile)
        logger.debug(f"Saving time: {time() - save_time}")

        selection_time = time()
        if isinstance(drones[0].controller, torch.nn.Module):
            new_population = drones
        else:
            new_population = selection_and_crossover(drones, p_tournament, p_crossover)
        logger.debug(f"Selection time: {time() - selection_time}")

        mutation_time = time()
        for i, drone in enumerate(new_population):
            drone.reset()
            drone.mutate(mutation_rate)
        logger.debug(f"Mutation time: {time() - mutation_time}")

        logger.info(
            f"Generation {gen}: Best Score: {best_score}, Global Best: {global_best_score}"
        )
        logger.info(
            f"\tCollected coins by best drone: {global_best_drone.coins}, best drone crashed: {global_best_drone.crashed}, time in air: {global_best_drone.flight_time}"
        )
        logger.info(
            f"Final distance between best drone and coin: {global_best_drone.distance_to_coin}\n"
        )

        # TODO: elitism doesn't seem to work correctly
        # Or otherwise, double-check if new coin positions are random
        # NOTE: set_coin_delta in the flight module does indeed contain some randomness:
        # set_coin_delta() produces a somewhat slightly random angle phi
        # for a new coin position.
        drones = new_population
        drones[0] = global_best_drone
        drones[0].reset()

        logger.info(f"Scores: {scores}")
        logger.info(f"Times: {times}")

    thread_pool.terminate()
    thread_pool.close()

    winning_drone_json = jsonpickle.encode(global_best_drone)
    with open(f"evolution/winning_drone_{execution_time}.json", "w") as jsonfile:
        json.dump(winning_drone_json, jsonfile)

    logger.info(f"To reference this training, use execution time {execution_time}")
