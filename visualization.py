"""
Created by Jan Schiffeler at 06.12.19
jan.schiffeler[at]gmail.com

Changed by
Marvin Becker

TODO: play from file option
TODO: split in 2 figures

Python 3.6.5
Library version:


"""


import socket
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import time as tm
import sys

host = 'localhost'
port = 65432
if len(sys.argv) > 1:
    port = int(sys.argv[1])
printouts = True
last_states = 20


class Plotter:
    """
    The class plotter opens a matplotlib window and connects to a tcpip server. Information received from the tcpip
    server is visualized in the plots. There are two modes for the plotwindow. "rotation" and "translation" which
    are set by the argument "showing". Rotation will plot thrusters, the three angles of rotation and wind in x
    direction. Translation will show the thrusters, a grid plot with the position of the drone, a rotating wind vector
    and the height in a line plot. All will have a white space in the top left, where the vtk window can be placed.
    """
    def __init__(self, host: str, port: int, printouts: bool, showing: str, n_last_states: int):

        # debug messages
        self.printouts = printouts

        # setpoint and plot dictionary
        self.setpoint_handler = {}
        self.plots = {}

        # data storage
        self.plotting_assistant = np.arange(n_last_states)
        self.rotation = np.zeros([n_last_states, 3], dtype=np.float32)
        self.translation = np.zeros([n_last_states, 3], dtype=np.float32)
        self.wind = np.zeros([n_last_states, 2], dtype=np.float32)

        # setup socket
        self._open_socket(host, port)

        # plot setup
        if self.printouts: print("[INFO] Setting up plots")
        if showing == "rotations":
            # assing functions to the translation version of the function
            self._update_plots = self._update_rotations
            self._return_plots = self._return_rotation
            self._setpoints = self._setpoints_rotation

            # create figure and layout
            self.figure = plt.figure(constrained_layout=True, figsize=(18.8, 10.0))
            grid = self.figure.add_gridspec(4, 4)

            # init angle plots
            self.roll_plot, roll_sp = self._setup_lineplot(grid=grid, row=0, start_column=2, end_column=4, label="Roll",
                                                  set_point=0, range_lower=-180, range_upper=180, color='r',
                                                  x_size=n_last_states)
            self.roll_plot.set(title="Time: 0s")
            self.roll_handle, = self.roll_plot.plot(0, 0, c='r')

            pitch_plot, pitch_sp = self._setup_lineplot(grid=grid, row=1, start_column=2, end_column=4, label="Pitch",
                                              set_point=0, range_lower=-180, range_upper=180,  color='g',
                                              x_size=n_last_states)
            self.pitch_handle,  = pitch_plot.plot(0, 0, c='g')

            yaw_plot, yaw_sp = self._setup_lineplot(grid=grid, row=2, start_column=2, end_column=4, label="Yaw",
                                            set_point=0, range_lower=-180, range_upper=180, color='b',
                                            x_size=n_last_states)
            self.yaw_handle, = yaw_plot.plot(0, 0, c='b')

            # init wind plot
            wind_plot = self._setup_lineplot(grid=grid, row=3, start_column=2, end_column=4, label="Wind",
                                             set_point=None, range_lower=-50, range_upper=50, color='y',
                                             x_size=n_last_states)
            self.wind_handle, = wind_plot.plot(0, 0, c='y')

            # init thruster plot
            self.thruster_plot = self._setup_barplot(grid=grid, label="Thrusters", set_point=0)
            self.thruster_handle = self.thruster_plot.bar([0.5, 1.5, 2.5, 3.5], [0.0, 0.0, 0.0, 0.0], color='b')

            # add setpoint handler to dictionary
            self.setpoint_handler.update({"roll": roll_sp, "pitch": pitch_sp, "yaw":yaw_sp})
            self.setpoint_handler.update({"roll": self.roll_plot, "pitch": pitch_plot, "yaw":yaw_plot})

            plt.draw()

        elif showing == "translations":
            # assing functions to the translation version of the function
            self._update_plots = self._update_translations
            self._return_plots = self._return_translation
            self._setpoints = self._setpoints_translation

            # create figure and layout
            self.figure = plt.figure(constrained_layout=True, figsize=(18.8, 10.0))
            grid = self.figure.add_gridspec(4, 4)

            # def wind arrow properties
            self.arrow_center = np.array([75, -75])
            self.arrow_length = 20

            # init gridplot with the arrows and plots on top
            self.plane_plot, self.wind_text, self.setpoint_xy = self._setup_gridplot(grid=grid, target=(90, 80),
                                                                                     size=100)
            xy = (60, -75)
            dxy = (20, 0)
            self.wind_arrow_handle = self.plane_plot.arrow(*xy, *dxy, color='y', head_width=6, width=2,
                                                           head_starts_at_zero=True)
            self.direction_arrow_handle = self.plane_plot.arrow(0, 0, 1, 0, color='g', head_width=4, width=2,
                                                                head_starts_at_zero=True)
            self.position_handle = self.plane_plot.scatter(0, 0, c='g')
            self.position_history_handle, = self.plane_plot.plot(self.translation[:, 0], self.translation[:, 1], c='r',
                                                                 alpha=0.5)

            # init height plot
            height_plot, height_sp = self._setup_lineplot(grid=grid, row=3, start_column=2, end_column=4,
                                                          label="Height", set_point=0, range_lower=0, range_upper=100,
                                                          color='y', x_size=n_last_states)
            self.height_handle, = height_plot.plot(0, 0, c='r')

            # init thruster plot
            self.thruster_plot = self._setup_barplot(grid=grid, label="Thrusters", set_point=0)
            self.thruster_handle = self.thruster_plot.bar([0.5, 1.5, 2.5, 3.5], [0.0, 0.0, 0.0, 0.0], color='b')

            # add setpoint handler to dictionary
            self.setpoint_handler["height"] = height_sp
            self.plots["height"] = height_plot

            plt.draw()

        else:
            raise TypeError("There is no flag named: ", showing, " ! Please input rotations or translations")

    def loop(self, *args):
        try:
            data = self.socket.recv(1024)
            received = data.decode()
            if printouts: print("[INFO] Message received: ", received)
            if 'quit' in received:
                self.socket.close()
                print("[INFO] Socket got closed")
                quit()
            else:
                print(self._decode_message(message=received))
                time, info = self._decode_message(message=received)
                if type(time) == float:
                    rot, trans, thrusters, wind = info
                    self._update_plots(time=time, thrusters=thrusters)
                    self._store_new_data(rotation=rot,
                                         translation=trans, wind=wind)
                else:
                    self._setpoints(info)

                return self._return_plots

        except OSError:
            # This is ugly but this should not happen in the real tests
            print("[ERROR] Socket got closed")
            quit()

    # PLOT SETUPS
    def _setup_barplot(self, grid, label: str, set_point: float) -> plt.axes:
        ax = self.figure.add_subplot(grid[3, :2])
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
        # ax.axhline(set_point, c='r', alpha=0.7)
        ax.axis([0, 4, 0, 101])
        return ax

    def _setup_gridplot(self, grid, target: tuple, size: int) -> (plt.axes, plt.text, None):
        ax = self.figure.add_subplot(grid[0:3, 2:4])

        major_ticks = np.arange(-size, size + 1, size/4)
        minor_ticks = np.arange(-size, size + 1, size/8)

        ax.set(title="Time: 0s")
        ax.set_xticks(major_ticks)
        ax.set_xticks(minor_ticks, minor=True)
        ax.set_yticks(major_ticks)
        ax.set_yticks(minor_ticks, minor=True)
        ax.grid(which='minor', alpha=0.5)
        ax.grid(which='major', alpha=0.8)

        # setpoint
        target_point = ax.scatter(*target, c='r')
        # axis limit
        ax.axis([-size -1, size + 1, -size - 1, size + 1])
        circ = plt.Circle(tuple(self.arrow_center), self.arrow_length / 2, fill=False)
        ax.add_artist(circ)
        texter = ax.text(55, -60, "Windspeed: 0 m/s")

        return ax, texter, target_point

    def _setup_lineplot(self, grid, row: int, start_column: int, end_column: int, label: str, set_point: float,
                        range_upper: int, range_lower: int, color: str, x_size: int) -> (plt.axes, plt.Line2D):
        ax = self.figure.add_subplot(grid[row, start_column:end_column])
        ax.set(ylabel=label)
        y_ticks = np.arange(range_lower, range_upper + 1, int((range_upper - range_lower) / 2))
        x_ticks = np.arange(0, 1, 1)
        ax.set_xticks(x_ticks)
        ax.set_yticks(y_ticks)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.grid(which='major', alpha=1)
        # setpoint
        if set_point is not None:
            target = ax.axhline(y=set_point, lw=1, c=color)
        ax.axis([0, x_size + 10, range_lower - 10, range_upper + 10])
        return ax, target

    # ROTATION FUNCTIONS
    def _update_rotations(self, time: float, thrusters: tuple):
        # update line plots
        self.roll_handle.set_data(self.plotting_assistant, self.rotation[:, 0])
        self.roll_plot.set(title="Time: {}s".format(time))
        self.pitch_handle.set_data(self.plotting_assistant, self.rotation[:, 1])
        self.yaw_handle.set_data(self.plotting_assistant, self.rotation[:, 2])
        self.wind_handle.set_data(self.plotting_assistant, self.wind[:, 0])

        # update bar plot
        self.thruster_handle.remove()
        self.thruster_handle = self.thruster_plot.bar([0.5, 1.5, 2.5, 3.5], thrusters, color='b')

    def _return_rotation(self):
        return self.roll_handle, self.pitch_handle, self.yaw_handle, self.wind_handle, self.thruster_handle

    def _setpoints_rotation(self, info):
        self.setpoint_handler["roll"].remove()
        self.setpoint_handler["roll"] = self.plots["roll"].axhline(y=info[0], lw=1, c='r')
        self.setpoint_handler["pitch"].remove()
        self.setpoint_handler["pitch"] = self.plots["pitch"].axhline(y=info[1], lw=1, c='g')
        self.setpoint_handler["yaw"].remove()
        self.setpoint_handler["yaw"] = self.plots["yaw"].axhline(y=info[2], lw=1, c='b')

    # TRANSLATION FUNCTIONS
    def _update_translations(self, time: float, thrusters: tuple):
        # update line and scatter plots
        self.plane_plot.set(title="Time: {}s".format(time))
        self.height_handle.set_data(self.plotting_assistant, self.translation[:, 2])

        # set wind arrow
        xy, dxy = self._arrow_position()
        self.wind_text.set_text("Windspeed: {:.2f} m/s".format(np.linalg.norm(self.wind[-1, :])))
        self.wind_arrow_handle.remove()
        self.wind_arrow_handle = self.plane_plot.arrow(*xy, *dxy, color='y', head_width=6, width=2,
                                                       head_starts_at_zero=True)

        # set position
        self.direction_arrow_handle.remove()
        xy = (self.translation[-1, 0], self.translation[-1, 1])
        dxy = (np.sin(self.rotation[-1, 2]) * self.arrow_length/2, np.cos(self.rotation[-1, 2]) * self.arrow_length/2)
        self.direction_arrow_handle = self.plane_plot.arrow(*xy, *dxy, color='g', head_width=6, width=2)
        self.position_handle.set_offsets(self.translation[-1, :2])
        self.position_history_handle.set_data(self.translation[:, 0], self.translation[:, 1])

        # update bar plot
        self.thruster_handle.remove()
        self.thruster_handle = self.thruster_plot.bar([0.5, 1.5, 2.5, 3.5], thrusters, color='b')

    def _return_translation(self):
        return self.thruster_handle, self.direction_arrow_handle, self.position_handle, self.position_history_handle, \
               self.wind_arrow_handle, self.wind_text, self.height_handle

    def _setpoints_translation(self, info):
        self.setpoint_handler["height"].remove()
        self.setpoint_handler["height"] = self.plots["height"].axhline(y=info[3], lw=1, c='r')
        self.setpoint_xy.setoffsets(info[4])

    def _arrow_position(self):
        wind = 1 / np.linalg.norm(self.wind[-1, :]) * self.wind[-1, :]
        xy = (self.arrow_center - wind * self.arrow_length / 2 * 1.6)
        dxy = (self.arrow_center + wind * self.arrow_length / 2 * 0.2) - xy
        return xy, dxy

    # SOCKET FUNCTIONS
    def _open_socket(self, host: str, port: int):
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
            quit()
        else:
            if self.printouts: print("[INFO] Connected")

    def _decode_message(self, message: str):
        # if self.printouts: print("message: " + message)
        msg = message.split("\n")[-2]
        meaning = msg.split(" ")
        if self.printouts: print("message: " + msg)
        if meaning[0] == "SETPOINTS":
            roll, pitch, yaw, x, y, z = [float(meaning[2 * i + 1]) for i in range(1, int(len(meaning) / 2))]
            return roll, pitch, yaw, z, (x, y)
        else:
            try:
                time, roll, pitch, yaw, x, y, z, t1, t2, t3, t4, wind_x, wind_y, wind_z = \
                    [float(meaning[2 * i + 1]) for i in range(int(len(meaning) / 2))]
                thrust = (t1 * 100, t2 * 100, t3 * 100, t4 * 100)
                return time, np.array([[roll, pitch, yaw]]) * 180 / np.pi, np.array([[x, y, z]]), thrust, \
                    np.array([[wind_x, wind_y]])
            except ValueError:
                if self.printouts: print("[ERROR] Couldn't decode message")
                return 0, (0, 0, 0), (0, 0, 0), (0, 0, 0, 0), (0, 0)

    # OTHER
    def _store_new_data(self, rotation: np.ndarray, translation: np.ndarray, wind: float):
        self.rotation = np.roll(self.rotation, -1, axis=0)
        self.rotation[-1, :] = rotation
        self.translation = np.roll(self.translation, -1, axis=0)
        self.translation[-1, :] = translation
        self.wind = np.roll(self.wind, -1, axis=0)
        self.wind[-1, :] = wind


if __name__ == "__main__":
    tm.sleep(1)
    dh = Plotter(host=host, port=port, printouts=printouts, showing="translations", n_last_states=last_states)

    print("[INFO] Starting the animation")
    anim = anim.FuncAnimation(dh.figure, dh.loop, interval=1, cache_frame_data=True)
    plt.show()

