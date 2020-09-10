import sys
import time
import json
import jsonpickle
from drone.drone import Drone
from flight import fly

if __name__ == "__main__":
    timestamp = sys.argv[1]
    port = 65432 
    if len(sys.argv) >= 3:
        port = int(sys.argv[2])

    with open(f"evolution/best_drone_{timestamp}.json") as jsonfile:
        jsonstring = json.load(jsonfile)
        drone = jsonpickle.decode(jsonstring, classes=Drone)
    drone.reset()
    drone.enable_visualization(True, 2, port)
    # drone.dh = DataHandler(
    #         parentfolder="./rewatch",
    #         visualize=True,
    #         n_servers=2,
    #         port=port,
    #     )
    # print(drone.controller.layer_spec)
    # print(drone.controller.__dict__)

    time.sleep(2)
    fly(drone, n_servers=1, visualize=True)

    print(f"Score: {drone.reward}")
    print(
        f"\tCollected coins by drone: {drone.coins}, best drone crashed: {drone.crashed}, time in air: {drone.flight_time}"
    )
