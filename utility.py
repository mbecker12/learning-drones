"""
Created by Jan Schiffeler at 17.12.19
jan.schiffeler[at]gmail.com

Changed by



Python 3.
Library version:


"""

import numpy as np


def stability_check(setpoints: np.ndarray, current_points: np.ndarray, **targets):
    dic = {"roll": 0, "pitch": 1, "yaw": 2, "x_y": None, "z": 5}
    stability = False
    for key, distance in targets.items():

        lookup = dic[key]
        if lookup is not None:
            if abs(setpoints[lookup, 0] - current_points[lookup, 0]) < distance:
                stability = True
            else:
                stability = False
                break
        else:
            vector_distance = setpoints[3:5, 0] - current_points[3:5, 0]
            if np.linalg.norm(vector_distance) < distance:
                stability = True
            else:
                stability = False
                break

    return stability


if __name__ == "__main__":
    setp = np.array([[10, 20, 30, 40, 50, 60]]).T
    current = np.array([[20, 30, 40, 50, 60, 70]]).T

    stab = stability_check(setp, current, pitch=10.1, x_y=20)
    print(stab)

    timer = 0
    setpoint_number = 0
    for i in range(100):
        if stability_check(setp, current, pitch=10.1, x_y=20):
            timer += 1
            if timer == 10:
                setpoint_number += 1
                timer = 0
        else:
             timer = 0

    print(setpoint_number)
