"""
Created by Jan Schiffeler at 06.12.19
jan.schiffeler[at]gmail.com

Changed by
Marvin Becker

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
import sys

import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


class DataHandler:
    """
    This class is called every iteration with it's function "new_data". The data is stored in arrays and the latest info
    is communicated to the visualization tool. After completion the function "finish" is called, it will write the data
    to .npy files and create one .csv overview file, also it closes the TCPIP socket.
    """
    def __init__(self, parentfolder: str = "", host: str = 'localhost', port: int = 65432, visualize: bool = True,
                 printouts: bool = True, n_servers: int = 2):

        self.n_servers = n_servers
        self.printouts = printouts
        self.dir_name = parentfolder + "/" + datetime.now().strftime("%d_%m_%Y_%H_%M_%S") + "/"

        # create folder to dump results
        try:
            os.makedirs(self.dir_name, exist_ok=True)
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
        self.wind = np.zeros([1, 2], dtype=np.float32)
        self.pid = np.zeros([1, 3], dtype=np.float32)
        self.setpoints = np.zeros([1, 6], dtype=np.float32)

    def new_data(self, time: float, rotation: np.ndarray, translation: np.ndarray, thrusters: np.ndarray,
                 wind: np.ndarray, pid: np.ndarray):
        """
        Create a new set of data points and visualize them
        :param time:
        :param rotation: [roll, pitch, yaw]
        :param translation: [x, y, z]
        :param thrusters: [t_1, t_2, t_3, t_4]
        :param wind: [w_x, w_y, w_z]
        :return:
        """

        # talk to visualization tools

        # check arguments
        if rotation.shape != (1, 3) or translation.shape != (1, 3) or thrusters.shape != (1, 4):
            logger.error("rotation.shape: " + str(rotation.shape))
            logger.error("translation.shape: " + str(translation.shape))
            logger.error("thrusters.shape: " + str(thrusters.shape))
            raise ValueError('One or more input values are not of the right size')

        # talk to visualization tool

        if self.visualize:
            self._send_message(time=time, rotation=rotation, translation=translation, thrusters=thrusters, wind=wind)

        # save data
        self.time = np.concatenate([self.time, np.array([[time]])], axis=0)
        self.rotation = np.concatenate([self.rotation, rotation], axis=0)
        self.translation = np.concatenate([self.translation, translation], axis=0)
        self.thrusters = np.concatenate([self.thrusters, thrusters], axis=0)
        self.wind = np.concatenate([self.wind, wind[0, :2]], axis=0)
        self.pid = np.concatenate([self.pid, pid], axis=0)

    def new_setpoints(self, rotation: np.ndarray, translation: np.ndarray):
        message = "SETPOINTS roll: {:.4f} pitch: {:.4f} yaw: {:.4f} ".format(*rotation[0])
        message += "x: {} y: {} z: {} ".format(*translation[0])
        if self.printouts: print("[INFO] Message send: ", message)
        for c in self.conn:
            try:
                c.sendall(message.encode())
            except BrokenPipeError:
                # if one connection fails save the progress and terminate
                print("[ERROR] One connection has been disconnected! Saving and closing...")
                self.finish()
        setpoints = np.repeat(a=np.array([[*rotation, *translation]]), repeats=self.time.shape[0], axis=0)
        self.setpoints = np.concatenate([self.setpoints, setpoints], axis=0)

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

        quit()

    def _open_server(self, host, port):
        if self.printouts: print("[INFO] Starting up server")
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind((host, port))
        except OSError as err:
            logger.error("host: " + str(host))
            logger.error("port: " + str(port))
            logger.exception(err)
            sys.exit()
        self.conn = []
        for i in range(self.n_servers):
            s.listen()
            c, addr = s.accept()
            self.conn.append(c)
            if self.printouts: print("[INFO] Client connected to: ", addr)

    def _send_message(self, time: float, rotation: np.ndarray, translation: np.ndarray,
                      thrusters: np.ndarray, wind: np.ndarray):
        message = "time: {} ".format(time)
        message += "roll: {:.4f} pitch: {:.4f} yaw: {:.4f} ".format(*rotation[0])
        message += "x: {} y: {} z: {} ".format(*translation[0])
        message += "t_1: {} t_2: {} t_3: {} t_4: {} ".format(*thrusters[0])
        message += "w_x: {} w_y: {} w_z: {}\n".format(*wind[0])
        if self.printouts: print("[INFO] Message send: ", message)
        for c in self.conn:
            try:
                c.sendall(message.encode())
            except BrokenPipeError:
                # if one connection fails save the progress and terminate
                print("[ERROR] One connection has been disconnected! Saving and closing...")
                self.finish()

    def _close_socket(self):
        for i in range(len(self.conn)):
            try:
                print(i)
                self.conn[i].sendall("quit".encode())
                self.conn[i].close()
            except BrokenPipeError:
                print("[ERROR] One connection has been disconnected before closing!")
                try:
                    print((i - 1) * (-1))
                    self.conn[(i - 1) * (-1)].sendall("quit".encode())
                    self.conn[(i - 1) * (-1)].close()
                except OSError:
                    print("Shit just got real. Abort mission. Abort mission!!!! Kkkkrrrrchh, Schriiimpff.")
        if self.printouts: print("[INFO] Closing sockets")

    def _save_npy(self):
        np.save(self.dir_name + "time", self.time)
        np.save(self.dir_name + "thrust_values", self.thrusters)
        np.save(self.dir_name + "rotation", self.rotation)
        np.save(self.dir_name + "translation", self.translation)
        np.save(self.dir_name + "wind", self.wind)
        np.save(self.dir_name + "pid", self.pid)
        np.save(self.dir_name + "setpoints", self.setpoints)
        if self.printouts: print("[INFO] .npy saved")

    def _save_csv(self):
        results = np.concatenate([self.time,
                                  self.rotation * 180/np.pi, self.translation,
                                  self.thrusters, self.wind, self.pid, self.setpoints], axis=1)
        np.savetxt(self.dir_name + "Results.csv", results, delimiter=",",
                   header='Time, Roll, Pitch, Yaw, X, Y, Z, T1, T2, T3, T4, WindX, WindY, WindZ,'
                          ' PID Roll, PID Pitch, PID Yaw, Set Roll, Set Pitch, Set Yaw, Set X, Set Y, Set Z')
        if self.printouts: print("[INFO] .csv saved")


if __name__ == "__main__":
    import time as tm
    dh = DataHandler(parentfolder="results", visualize=True)
    roll, pitch, yaw, x, y, z = [np.random.randint(-50, 50) for i in range(6)]
    trans = np.array([[x, y, z]])
    rot = np.array([[roll, pitch, yaw]]) * np.pi/180
    for t in range(400):
        rot += np.random.randint(-10, 10, [1, 3]) * np.pi/180
        thrust = np.random.random([1, 4])
        w = np.random.randint(-10, 10, [1, 3])

        dh.new_data(time=t, rotation=rot,
                    translation=trans,
                    thrusters=thrust, wind=w)
        tm.sleep(0.1)

    dh.finish()
