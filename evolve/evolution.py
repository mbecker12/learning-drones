import torch
import numpy as np
from copy import deepcopy
import sys
import yaml
from time import sleep, time
import traceback
import os
import tracemalloc

tracemalloc.start(100)

# possibly bad style, but it works...
sys.path.append(os.getcwd())

from drone.drone import Drone
from evolve.brain import DroneBrain
from evolve.util import (
    selection_and_crossover,
    tournament_select,
    get_best_of_generation,
    avg_of_list,
)

from flight import fly
import jsonpickle
import json
from multiprocessing.pool import ThreadPool, Pool
from multiprocessing import Manager
import logging

# "evolution_log",
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("evolution")
logger.setLevel(logging.INFO)
# from multiprocessing import shared_memory

n_weigths = 75520
n_generations = 1000
n_drones = 40
initial_mutation_rate = (
    100 / 75520
)  # aim to mutate 10 genes/weights in the whole chromosome
mutation_rate0 = 50 / 75520  # aim to mutate 5 weights...

initial_mutation_rate = 0.01  # mutate a certain percentage of weights
mutation_rate0 = 0.01  # mutate a certain percentage of weights

mutation_rate_decay = 0.999
p_tournament = 0.8
p_crossover = 0.3  # is ignored for now with the torch implementation
n_executions = 8  # number of games one individual performs in one generation
max_timesteps = 1200


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
    time1 = tracemalloc.take_snapshot()
    init_time = time()
    execution_time = int(time())
    logger.info(f"Starting neuroevolution with execution time {execution_time}")
    drones = [Drone(visualize=False, n_servers=0) for _ in range(n_drones)]
    avg_scores = [0] * n_drones
    min_scores = [0] * n_drones
    times = [0] * n_drones

    with open("choreo.yaml") as choreo_file:
        choreo = yaml.load(choreo_file)
    train_choreos = choreo["train"]
    val_choreos = choreo["val"]

    for i, drone in enumerate(drones):
        # mutate all pretrained drones once, otherwise they all score the same
        drone.reset()
        drone.reset_accumulated_stats()
        drone.mutate(initial_mutation_rate)

    global_best_score = -np.Infinity
    global_best_drone = None

    global_best_min_score = -np.Infinity
    global_best_min_drone = None

    thread_pool = ThreadPool(1)

    logger.debug(f"Initialization time: {time() - init_time}")
    start_time = time()
    for gen in range(n_generations):
        generation_best_score = -np.Infinity
        generation_best_drone = None

        mutation_rate = mutation_rate0 * mutation_rate_decay ** gen
        generation_time = time() - start_time
        logger.info(f"Time elapsed for last generation: {generation_time}\n")
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
            min_score = np.Inf
            avg_flight_time = 0
            for j in range(n_executions):
                score, flight_time, idx = fly(
                    drone,
                    idx=i,
                    run_idx=j,
                    total_runs=n_executions,
                    timesteps=max_timesteps,
                    choreo=train_choreos[j],
                )
                avg_score += score
                avg_flight_time += flight_time
                drone.reset()

                if score < min_score:
                    min_score = score
                # TODO: implement elitism with minimum score as well
                # Credits to Elsa

            avg_scores[idx] = avg_score / n_executions
            min_scores[idx] = min_score
            times[idx] = avg_flight_time / n_executions
        logger.debug(f"Fly time: {time() - fly_time}")
        best_avg_index = np.argmax(avg_scores)
        best_avg_score = avg_scores[best_avg_index]

        best_min_index = np.argmax(min_scores)
        best_min_score = min_scores[best_min_index]
        save_time = time()

        if best_avg_score > global_best_score:
            logger.info(f"Found new global best! At index {best_avg_index}")
            global_best_score = best_avg_score
            # global_best_drone = deepcopy(drones[best_index])
            global_best_drone = deepcopy(drones[best_avg_index])
            global_best_network = deepcopy(drones[best_avg_index].controller)

            # best_drone_json = jsonpickle.encode(global_best_drone)
            best_network_json = jsonpickle.encode(global_best_network)
            with open(f"evolution/best_network_{execution_time}.json", "w") as jsonfile:
                json.dump(best_network_json, jsonfile)
        if best_min_score > global_best_min_score:
            logger.info(f"Found new global best minimum! At index {best_min_index}")
            global_best_min_score = best_min_score
            # global_best_drone = deepcopy(drones[best_index])
            global_best_min_drone = deepcopy(drones[best_min_index])
            global_best_min_network = deepcopy(drones[best_min_index].controller)

            # best_drone_json = jsonpickle.encode(global_best_drone)
            best_min_network_json = jsonpickle.encode(global_best_min_network)
            with open(
                f"evolution/best_min_network_{execution_time}.json", "w"
            ) as jsonfile:
                json.dump(best_min_network_json, jsonfile)
        logger.debug(f"Saving time: {time() - save_time}")
        # global_best_drone = Drone(visualize=False)
        # best_index = 0
        selection_time = time()
        if isinstance(drones[0].controller, torch.nn.Module):
            new_population = deepcopy(drones)
        else:
            new_population = selection_and_crossover(drones, p_tournament, p_crossover)
        logger.debug(f"Selection time: {time() - selection_time}")

        logger.info(
            f"Generation {gen}: Best Avg Score: {best_avg_score}, Global Best: {global_best_score}"
        )
        logger.info(
            f"Generation {gen}: Best Min Score: {best_min_score}, Global Best: {global_best_min_score}"
        )

        logger.info(
            f"\t\tCollected coins by generation best drone: {avg_of_list(drones[best_avg_index].coins_collected)}, best drone crashed: {avg_of_list(global_best_drone.runs_crashed)}, time in air: {avg_of_list(global_best_drone.flight_times)}"
        )
        logger.info(
            f"\t\tFinal dist. generation best drone - coin: {avg_of_list(drones[best_avg_index].distances_to_coin)}"
        )
        logger.info(
            f"\t\tCollected coins by global best drone:     {avg_of_list(global_best_drone.coins_collected)}, best drone crashed: {avg_of_list(global_best_drone.runs_crashed)}, time in air: {avg_of_list(global_best_drone.flight_times)}"
        )
        logger.info(
            f"\t\tFinal dist. global best drone - coin:     {avg_of_list(global_best_drone.distances_to_coin)}"
        )
        if (gen + 1) % 5 == 0:
            logger.info(
                f"To reference this training, use execution time {execution_time}"
            )

        mutation_time = time()
        for i, drone in enumerate(new_population):
            drone.mutate(mutation_rate)
            drone.reset_accumulated_stats()

        logger.debug(f"Mutation time: {time() - mutation_time}")
        # TODO: elitism doesn't seem to work correctly
        # Or otherwise, double-check if new coin positions are random
        # NOTE: set_coin_delta in the flight module does indeed contain some randomness:
        # set_coin_delta() produces a somewhat slightly random angle phi
        # for a new coin position.

        elitism_time = time()
        drones = deepcopy(new_population)
        del new_population

        with open(f"evolution/best_network_{execution_time}.json") as jsonfile:
            jsonstring = json.load(jsonfile)
            drone_nn = jsonpickle.decode(jsonstring, classes=DroneBrain)
            drones[0].controller = drone_nn
            drones[0].reset()

        with open(f"evolution/best_min_network_{execution_time}.json") as jsonfile:
            jsonstring = json.load(jsonfile)
            drone_nn = jsonpickle.decode(jsonstring, classes=DroneBrain)
            drones[1].controller = drone_nn
            drones[1].reset()

        # drones[0] = global_best_drone
        # drones[0].reset()
        logger.debug(f"Elitism time: {time() - elitism_time}")
        logger.info(f"Avg Scores: {avg_of_list(avg_scores)}")
        logger.info(f"Avg Flight Times: {avg_of_list(times)}")

    thread_pool.terminate()
    thread_pool.close()

    winning_drone_json = jsonpickle.encode(global_best_drone)
    with open(f"evolution/winning_drone_{execution_time}.json", "w") as jsonfile:
        json.dump(winning_drone_json, jsonfile)

    # logger.info(f"To reference this training, use execution time {execution_time}")
