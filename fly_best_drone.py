import sys
import time
import json
import jsonpickle
from drone.drone import Drone
from flight import fly

if __name__ == "__main__":
    timestamp = sys.argv[1]

    with open(f"evolution/best_drone_{timestamp}.json") as jsonfile:
        jsonstring = json.load(jsonfile)
        drone = jsonpickle.decode(jsonstring, classes=Drone)
    drone.reset()
    drone.enable_visualization(True, 2, 65432)
    # print(drone.controller.layer_spec)
    # print(drone.controller.__dict__)

    time.sleep(2)
    fly(drone, n_servers=1, visualize=True)

    print(f"Score: {drone.reward}")
    print(
        f"\tCollected coins by drone: {drone.coins}, best drone crashed: {drone.crashed}, time in air: {drone.flight_time}"
    )
