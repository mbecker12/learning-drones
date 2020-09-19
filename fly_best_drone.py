import sys
import time
import json
import jsonpickle
from drone.drone import Drone
from drone.onboard_computer import StraightAhead 
from flight import fly

if __name__ == "__main__":
    if len(sys.argv) > 1:
        timestamp = sys.argv[1]
    else:
        timestamp = None
    port = 65432 
    if len(sys.argv) >= 3:
        port = int(sys.argv[2])

    print(f"timestamp: {timestamp}")
    print(f"port: {port}")

    if timestamp is not None and timestamp != "123":
        with open(f"evolution/best_drone_{timestamp}.json") as jsonfile:
            jsonstring = json.load(jsonfile)
            drone = jsonpickle.decode(jsonstring, classes=Drone)
        drone.reset()
        drone.enable_visualization(True, 2, port)
    else:
        print(f"Initialize 'StraightAhead' controller")
        drone = Drone(port=port)
        drone.controller = StraightAhead(None, None)
    # drone.dh = DataHandler(
    #         parentfolder="./rewatch",
    #         visualize=True,
    #         n_servers=2,
    #         port=port,
    #     )
    # print(drone.controller.layer_spec)
    # print(drone.controller.__dict__)


    time.sleep(2)
    fly(drone, n_servers=2, visualize=True)

    print(f"Score: {drone.reward}")
    print(
        f"\tCollected coins by drone: {drone.coins}, best drone crashed: {drone.crashed}, time in air: {drone.flight_time}"
    )
