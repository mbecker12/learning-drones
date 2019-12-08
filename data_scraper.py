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
import time as tm


class DataHandler:
    """
    This class is called every iteration with it's function "new_data". The data is stored in arrays and the latest info
    is communicated to the visualization tool. After completion the function "finish" is called, it will write the data
    to .npy files and create one .csv overview file, also it closes the TCPIP socket.
    """
    def __init__(self, parentfolder: str = "", host: str = 'localhost', port: int = 65432, visualize: bool = True,
                 printouts: bool = True):

        self.printouts = printouts
        self.dir_name = parentfolder + "/" + datetime.now().strftime("%d_%m_%Y_%H_%M_%S") + "/"

        # create folder to dump results
        try:
            os.mkdir(self.dir_name)
        except OSError:
            print("[ERROR] Creation of the directory {} failed".format(self.dir_name),
                  ". Parentfolders need to be manually created!")
            quit()
        else:
            if self.printouts: print("[INFO] Created the directory {} ".format(self.dir_name))

        # socket info
        self.visualize = visualize
        if self.visualize:
            self._open_server(host, port)

        # data arrays
        self.time = np.zeros([1, 1], dtype=np.float32)
        self.thrusters = np.zeros([1, 4], dtype=np.float32)
        self.rotation = np.zeros([1, 3], dtype=np.float32)
        self.translation = np.zeros([1, 3], dtype=np.float32)
        self.wind = np.zeros([1, 1], dtype=np.float32)

    def new_data(self, time: float, rotation: np.ndarray, translation: np.ndarray, thrusters: np.ndarray, wind: float):
        """
        Create a new set of data points and visualize them
        :param time:
        :param rotation: [roll, pitch, yaw]
        :param translation: [x, y, z]
        :param thrusters: [t_1, t_2, t_3, t_4]
        :param wind:
        :return:
        """
        # talk to visualization tools
        if self.visualize:
            self._send_message(time=time, rotation=rotation, translation=translation, thrusters=thrusters, wind=wind)

        # save data
        self.time = np.concatenate([self.time, np.array([[time]])], axis=0)
        self.rotation = np.concatenate([self.rotation, rotation], axis=0)
        self.translation = np.concatenate([self.translation, translation], axis=0)
        self.thrusters = np.concatenate([self.thrusters, thrusters], axis=0)
        self.wind = np.concatenate([self.wind, np.array([[wind]])], axis=0)

    def finish(self):
        """
        Close sockets and save data to files
        :return:
        """
        # close socket
        if self.visualize:
            self._close_socket()
            if self.printouts: print("[INFO] Socket closed")

        # remove initial zero
        self.time = self.time[1:, :]
        self.rotation = self.rotation[1:, :]
        self.translation = self.translation[1:, :]
        self.thrusters = self.thrusters[1:, :]
        self.wind = self.wind[1:, :]

        # save data
        self._save_csv()
        self._save_npy()

    def _open_server(self, host, port):
        if self.printouts: print("[INFO] Starting up server")
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((host, port))
        self.conn = []
        for i in range(2):
            s.listen()
            c, addr = s.accept()
            self.conn.append(c)
            if self.printouts: print("[INFO] Client connected to: ", addr)

    def _send_message(self, time: float, rotation: np.ndarray, translation: np.ndarray,
                      thrusters: np.ndarray, wind: float):
        message = "time: {} ".format(time)
        message += "roll: {} pitch: {} yaw: {} ".format(*rotation[0])
        message += "x: {} y: {} z: {} ".format(*translation[0])
        message += "t_1: {} t_2: {} t_3: {} t_4: {}, ".format(*thrusters[0])
        message += "v_w: {}".format(wind)
        for c in self.conn:
            c.sendall(message.encode())

    def _close_socket(self):
        for c in self.conn:
            c.sendall("quit".encode())
            c.close()

    def _save_npy(self):
        np.save(self.dir_name + "time", self.time)
        np.save(self.dir_name + "thrust_values", self.thrusters)
        np.save(self.dir_name + "rotation", self.rotation)
        np.save(self.dir_name + "translation", self.translation)
        np.save(self.dir_name + "wind", self.wind)
        if self.printouts: print("[INFO] .npy saved")

    def _save_csv(self):
        results = np.concatenate([self.time,
                                  self.rotation * 180/np.pi, self.translation,
                                  self.thrusters, self.wind], axis=1)
        np.savetxt(self.dir_name + "Results.csv", results, delimiter=",",
                   header='Time, Roll, Pitch, Yaw, X, Y, Z, T1, T2, T3, T4, Windspeed')
        if self.printouts: print("[INFO] .csv saved")


if __name__ == "__main__":
    dh = DataHandler(parentfolder="results", visualize=True)
    dh.new_data(time=0, rotation=np.array([[1, 2, 3]]),
                translation=np.array([[2, 3, 1]]),
                thrusters=np.array([[1, 4.0, 3, 5.0]]), wind=50.341)
    input("waiting fooooor me baby")
    dh.new_data(time=0, rotation=np.array([[1, 2, 3]]),
                translation=np.array([[2, 3, 1]]),
                thrusters=np.array([[1, 4.0, 3, 5.0]]), wind=50.341)
    input("waiting fooooor me baby")
    dh.new_data(time=0, rotation=np.array([[1, 2, 3]]),
                translation=np.array([[2, 3, 1]]),
                thrusters=np.array([[1, 4.0, 3, 5.0]]), wind=50.341)
    input("waiting fooooor me baby")
    dh.new_data(time=0, rotation=np.array([[1, 2, 3]]),
                translation=np.array([[2, 3, 1]]),
                thrusters=np.array([[1, 4.0, 3, 5.0]]), wind=50.341)
    input("waiting fooooor me baby")
    dh.finish()
