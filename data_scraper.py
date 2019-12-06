"""
Created by Jan Schiffeler at 06.12.19
jan.schiffeler[at]gmail.com

Changed by

communication string:

"time: 1
t_1: 0.5 t_2: 0.5 t_3: 0.5 t_4: 0.5
roll: 1.81 pitch: 1.81 yaw: 1.81
x: 1 y: 1 z: 1
v_w: 10"

decode with:
meaning = message.split(" ")
[meaning[2*i+1] for i in range(int(len(meaning)/2))]

Python 3.6.5
Library version:
numpy 1.17
socket, datetime standard libraries

"""
import numpy as np
import socket
from datetime import datetime
import os


class DataHandler:
    def __init__(self, parentfolder: str = "", host: str = 'localhost', port: int = 65432, visualize: bool = True):
        # create folder to dump results
        self.dir_name = parentfolder + datetime.now().strftime("%d_%m_%Y_%H_%M")
        try:
            os.mkdir(self.dir_name)
        except OSError:
            print("[ERROR] Creation of the directory {} failed".format(self.dir_name))
        else:
            print("[INFO] Created the directory {} ".format(self.dir_name))

        # socket info
        if visualize:
            print("[INFO] Waiting for connection")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((host, port))
            print("[INFO] Connected")

        # result arrays
        self.time = np.zeros(1)
        self.t_n = np.zeros([1, 4])
        self.rotation = np.zeros([1, 3])
        self.translation = np.zeros([1, 3])
        self.wind = np.zeros(1)

    def send_message(self, time: float, t_n: np.ndarray, rotation: np.ndarray, translation: np.ndarray, wind: float):
        message = "time: {} ".format(time)
        message += "t_1: {} t_2: {} t_3: {} t_4: {}, ".format(*t_n)
        message += "roll: {} pitch: {} yaw: {} ".format(*rotation)
        message += "x: {} y: {} z: {} ".format(*translation)
        message += "v_w: {}".format(wind)
        socket.sendall(message.encode())

    def new_data(self, time: float, t_n: np.ndarray, rotation: np.ndarray, translation: np.ndarray, wind: float):
        self.send_message(time=time, t_n=t_n, rotation=rotation, translation=translation, wind=wind)

    def close_socket(self):
        self.socket.close()

    def finish_run(self):
        self.close_socket
        self.save_csv()
        self.save_npy()

    def save_npy(self):
        pass

    def save_csv(self):
        pass



if __name__ == "__main__":
    dh = DataHandler(parentfolder="results/")
