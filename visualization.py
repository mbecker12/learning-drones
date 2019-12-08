"""
Created by Jan Schiffeler at 06.12.19
jan.schiffeler[at]gmail.com

Changed by

TODO:
- initialization message with all the setpoints.
- translation scatter plots

Python 3.6.5
Library version:


"""


import socket
import numpy as np
import matplotlib.pyplot as plt
import time as tm

host = 'localhost'
port = 65432
printouts = True
last_states = 50


class Plotter:
    def __init__(self, host: str, port: int, printouts: bool, showing: str, n_last_states: int):

        self.printouts = printouts

        # data storage
        self.plotting_assistant = np.arange(n_last_states)
        self.rotation = np.zeros([n_last_states, 3], dtype=np.float32)
        self.translation = np.zeros([n_last_states, 3], dtype=np.float32)
        self.wind = np.zeros([n_last_states, 1], dtype=np.float32)

        # setup socket
        self._open_socket(host, port)

        # plot setup
        if showing == "rotations":
            self._update_plots = self._update_rotations

            fig = plt.figure(constrained_layout=True, figsize=(7.8, 10.0))
            grid = fig.add_gridspec(4, 4)

            self.roll_plot = self._setup_subplot(fig, grid, 0, 2, 4, "Roll", 110, 180, 'r')
            self.roll_plot.set(title="Time: 0s")
            self.roll_handle = self.roll_plot.plot(0, 0, c='r')

            pitch_plot = self._setup_subplot(fig, grid, 1, 2, 4, "Pitch", 110, 180, 'g')
            self.pitch_handle = pitch_plot.plot(0, 0, c='g')

            yaw_plot = self._setup_subplot(fig, grid, 2, 2, 4, "Roll", 110, 180, 'b')
            self.yaw_handle = yaw_plot.plot(0, 0, c='b')

            wind_plot = self._setup_subplot(fig, grid, 3, 0, 2, "Wind", None, 100, 'y')
            self.wind_handle = wind_plot.plot(0, 0, c='y')

            self.thruster_plot = self._setup_barplot(fig, grid, "Thrusters", 10)
            self.thruster_handle = self.thruster_plot.bar([0.5, 1.5, 2.5, 3.5], [0.0, 0.0, 0.0, 0.0], color='b')

            plt.draw()
        elif showing == "translations":
            self._update_plots = self._update_translations
            # DO ME BABY
        else:
            raise TypeError("There is no flag named: ", showing, " ! Please input rotations or translations")

    def loop(self):
        try:
            data = self.socket.recv(1024)
            received = data.decode()
            if printouts: print("[INFO] Message received: ", received)
            if received == 'quit':
                self.socket.close()
            else:
                time, roll, pitch, yaw, x, y, z, t1, t2, t3, t4, wind = self._decode_message(message=received)
                self._store_new_data(rotation=np.array([[roll, pitch, yaw]]),
                                     translation=np.array([[x, y, z]]), wind=wind)
                self._update_plots(time, t1, t2, t3, t4)

            return False

        except OSError:
            # This is ugly but this should not happen in the real tests
            print("[ERROR] Socket got closed")
            quit()

            return True

    def _open_socket(self, host, port):
        if self.printouts: print("[INFO] Waiting for connection")
        timer = 0
        while timer < 60:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((host, port))
                break
            except ConnectionRefusedError:
                timer += 1
                if self.printouts: print("[INFO] No server found! Try: ", timer, "/60")
                tm.sleep(0.5)
        if timer == 60:
            print("[ERROR] Connection could not be established. Will continue without visualization")
            self.visualize = False
        else:
            if self.printouts: print("[INFO] Connected")

    def _setup_barplot(self, figure, grid, label, sp):
        ax = figure.add_subplot(grid[3, :1])
        ax.set(xlabel=label)
        ticks = np.arange(0, 100 + 1, 50)

        ax.set_xticks(np.arange(0.5, 4.5, 1))
        ax.xaxis.set_ticks_position('none')
        ax.set_yticks(ticks)
        ax.set_yticks(ticks, minor=True)
        ax.grid(which='minor', alpha=0.8)
        ax.grid(which='major', alpha=0.0)

        ax.set(xticklabels=['T1', 'T2', 'T3', 'T4'])
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.axhline(sp, c='r', alpha=0.7)
        ax.axis([0, 4, 0, 101])
        return ax

    def _setup_subplot(self, figure, grid, row, s_c, e_c, label, sp, r, c):
        ax = figure.add_subplot(grid[row, s_c:e_c])
        ax.set(ylabel=label)
        y_ticks = np.arange(-r, r + 1, int(r / 2))
        x_ticks = np.arange(0, 1, 1)
        ax.set_xticks(x_ticks)
        ax.set_yticks(y_ticks)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.grid(which='major', alpha=1)
        # setpoint
        if sp is not None:
            ax.axhline(y=sp, lw=1, c=c)
        ax.axis([0, 20, -r - 10, r + 10])
        return ax

    def _store_new_data(self, rotation: np.ndarray, translation: np.ndarray, wind: float):
        rotation *= 180 / np.pi
        self.rotation = np.roll(rotation, 1, axis=1)[-1, :]
        self.translation = np.roll(translation, 1, axis=1)[-1, :]
        self.wind = np.roll(wind, 1, axis=1)[-1, :]

    def _update_rotations(self, time, t1, t2, t3, t4):
        # update line plots
        self.roll_handle.set_data(self.plotting_assistant, self.roll)
        self.roll_plot.set(title="Time: {}s".format(time))
        self.pitch_handle.set_data(self.plotting_assistant, self.pitch)
        self.yaw_handle.set_data(self.plotting_assistant, self.yaw)
        self.wind_handle.set_data(self.plotting_assistant, self.wind)

        # update bar plot
        self.thruster_handle.remove()
        self.thruster_handle = self.thruster_plot.bar([0.5, 1.5, 2.5, 3.5], [t1, t2, t3, t4])

    def _update_translations(self):
        pass

    def _decode_message(self, message: str):
        meaning = message.split(" ")
        return [float(meaning[2 * i + 1]) for i in range(int(len(meaning) / 2))]


dh = Plotter(host=host, port=port, printouts=printouts, showing="rotations", n_last_states=last_states)

while dh.loop():
    pass
