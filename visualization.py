"""
Created by Jan Schiffeler at 06.12.19
jan.schiffeler[at]gmail.com

Changed by

TODO:
- initialization message with all the setpoints.

Python 3.6.5
Library version:


"""


import socket
import vtk
import numpy as np
import matplotlib as plt

host = 'localhost'
port = 65432
printouts = True


class DroneHandle:
    def __init__(self, actor: vtk.vtkActor, host: str, port: int, printouts: bool, showing: str):
        self.actor = actor

        # data storage
        self.time = np.zeros([1, 1], dtype=np.float32)
        self.thrusters = np.zeros([1, 4], dtype=np.float32)
        self.rotation = np.zeros([1, 3], dtype=np.float32)
        self.translation = np.zeros([1, 3], dtype=np.float32)
        self.wind = np.zeros([1, 1], dtype=np.float32)

        # setup socket
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((host, port))
        self.s.listen()
        self.conn, addr = self.s.accept()
        if printouts: print('[INFO] Connected to', addr)

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

            thruster_plot = self._setup_barplot(fig, grid, "Thrusters", 10)
            self.thruster_handle = thruster_plot.bar([0.5, 1.5, 2.5, 3.5], [0.0, 0.0, 0.0, 0.0], color='b')
        elif showing == "translations":
            self._update_plots = self._update_translations
        else:
            raise TypeError("There is no flag named: ", showing, " ! Please input rotations or translations")


    def animate(self, obj, event):
        try:
            data = self.conn.recv(1024)
            received = data.decode()
            if printouts: print("[INFO] Message recieved: ", received)
            if received == 'quit':
                self.conn.close()
            else:
                time, roll, pitch, yaw, x, y, z, t1, t2, t3, t4, wind = self._decode_message(message=received)
                self._store_new_data(time=time, rotation=np.array([[roll, pitch, yaw]]),
                                     translation=np.array([[x, y, z]]),
                                     thrusters=np.array([[t1, t2, t3, t4]]), wind=wind)
                self._update_vtk(obj,
                                 d_roll=self.rotation[-1, 0] - self.rotation[-2, 0],
                                 d_pitch=self.rotation[-1, 1] - self.rotation[-2, 1],
                                 d_yaw=self.rotation[-1, 2] - self.rotation[-2, 2])
                self._update_plots(time, roll, pitch, yaw, x, y, z, t1, t2, t3, t4, wind)

        except OSError:
            # This is ugly but this should not happen in the real tests
            print("[ERROR] Socket got closed")
            quit()

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
        if not None:
            ax.axhline(y=sp, lw=1, c=c)
        ax.axis([0, 20, -r - 10, r + 10])
        return ax

    def _store_new_data(self, time: float, rotation: np.ndarray, translation: np.ndarray,
                        thrusters: np.ndarray, wind: float):
        self.time = np.concatenate([self.time, np.array([[time]])], axis=0)
        rotation *= 180 / np.pi
        self.rotation = np.concatenate([self.rotation, rotation], axis=0)
        self.translation = np.concatenate([self.translation, translation], axis=0)
        self.thrusters = np.concatenate([self.thrusters, thrusters], axis=0)
        self.wind = np.concatenate([self.wind, np.array([[wind]])], axis=0)

    def _update_rotations(self, time, roll, pitch, yaw, x, y, z, t1, t2, t3, t4, wind):
        self.roll_handle.set_data()
        self.roll_plot.set(title="Time: {}s".format(time))
        self.pitch_handle.set_data()

    def _update_translations(self):
        pass

    def _decode_message(self, message: str):
        meaning = message.split(" ")
        return [meaning[2 * i + 1] for i in range(int(len(meaning) / 2))]

    def _update_vtk(self, obj, d_roll: float, d_pitch: float, d_yaw: float):
        self.actor.RotateX(d_roll)
        self.actor.RotateY(d_pitch)
        self.actor.RotateZ(d_yaw)
        obj.GetRenderWindow().Render()


# create a rendering window and renderer
ren = vtk.vtkRenderer()
ren.SetBackground(0.0, 0.0, 0.0)

renWin = vtk.vtkRenderWindow()
renWin.SetSize(1600, 1600)
# renWin.SetWindowName("Please give me my drone!")
renWin.AddRenderer(ren)

# create a renderwindowinteractor
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)

drone = vtk.vtkPLYReader()
drone.SetFileName("Drone.ply")
drone.Update()

# mapper
droneMapper = vtk.vtkPolyDataMapper()
droneMapper.SetInputConnection(drone.GetOutputPort())

# actor
droneActor = vtk.vtkActor()
droneActor.SetMapper(droneMapper)
droneActor.GetProperty().SetColor(1.0, 0.0, 0.0)

axes = vtk.vtkAxesActor()
axes.SetTotalLength(30, 30, 30)
axes.SetXAxisLabelText("")
axes.SetYAxisLabelText("")
axes.SetZAxisLabelText("")

# camera
camera = vtk.vtkCamera()
camera.SetPosition(100, 100, 100)
camera.SetRoll(-120)
camera.SetFocalPoint(0, 0, 0)

# assign actor to the renderer
ren.AddActor(droneActor)
ren.AddActor(axes)
ren.SetActiveCamera(camera)

# enable user interface interactor
renWin.Render()

iren.Initialize()

dh = DroneHandle(actor=droneActor, host=host, port=port, printouts=printouts)
iren.AddObserver('TimerEvent', dh.animate)
iren.CreateRepeatingTimer(0)

iren.Start()