import numpy as np
from copy import deepcopy
import sys
from time import sleep, time
import traceback
import os
sys.path.append(os.getcwd())
#sys.path.append("/home/marvin/Projects/Drones")

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

# from multiprocessing import shared_memory

n_generations = 500
n_drones = 40
mutation_rate0 = 0.2
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

    thread_pool = ThreadPool(1)

    start_time = time()
    for gen in range(n_generations):
        mutation_rate = mutation_rate0 * mutation_rate_decay ** gen
        generation_time = time() - start_time
        print(f"Time elapsed for last generation: {generation_time}")
        start_time = time()
        # TODO: find a way to do this in separate threads

        # TODO: threads won't be the answer; need to use processes
        # TODO: For that, we need to find a way to share complex/nested objects between processes
        # maybe this will help:
        # https://docs.python.org/3/library/multiprocessing.html#proxy-objects
        # https://www.geeksforgeeks.org/multiprocessing-python-set-2/

        for i, drone in enumerate(drones):
            avg_score = 0
            avg_flight_time = 0
            for j in range(n_executions):
                score, flight_time, idx = fly(
                    drone, idx=i, run_idx=j, total_runs=n_executions
                )
                avg_score += score
                avg_flight_time += flight_time
                drone.reset()

            scores[idx] = avg_score / n_executions
            times[idx] = avg_flight_time / n_executions
            # execute function 'fly' from module flight in multiple threads
        # async_results = [thread_pool.apply_async(fly, (drone, ), {"idx": i}, error_callback=thread_error_prompt) for i, drone in enumerate(drones)]

        # return_val = async_result.get()
        # score, flight_time, idx = return_val

        # for result in async_results:
        #     score, flight_time, idx = result.get()
        #     # print(score, flight_time, idx)
        #     scores[idx] = score
        #     times[idx] = flight_time
        # async_result.get()
        # thread_pool.join()
        # best_index, best_score = get_best_of_generation(scores)
        best_index = np.argmax(scores)
        best_score = scores[best_index]

        if best_score > global_best_score:
            print(f"Found new global best! At index {best_index}")
            global_best_score = best_score
            global_best_drone = deepcopy(drones[best_index])

            best_drone_json = jsonpickle.encode(global_best_drone)
            with open(f"evolution/best_drone_{execution_time}.json", "w") as jsonfile:
                json.dump(best_drone_json, jsonfile)

        new_population = selection_and_crossover(drones, p_tournament, p_crossover)

        for i, drone in enumerate(new_population):
            drone.reset()
            drone.mutate(mutation_rate)

        print(
            f"Generation {gen}: Best Score: {best_score}, Global Best: {global_best_score}"
        )
        print(
            f"\tCollected coins by best drone: {global_best_drone.coins}, best drone crashed: {global_best_drone.crashed}, time in air: {global_best_drone.flight_time}"
        )
        print(
            f"Final distance between best drone and coin: {global_best_drone.distance_to_coin}"
        )
        print()
        drones = new_population
        drones[0] = global_best_drone
        drones[0].reset()

        print(f"Scores: {scores}")
        print(f"Times: {times}")
        print()

    thread_pool.terminate()
    thread_pool.close()

    winning_drone_json = jsonpickle.encode(global_best_drone)
    with open(f"evolution/winning_drone_{execution_time}.json", "w") as jsonfile:
        json.dump(winning_drone_json, jsonfile)

    print(f"To reference this training, use execution time {execution_time}")
