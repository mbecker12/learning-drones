"""
Created by Jan Schiffeler at 08.12.19
jan.schiffeler[at]gmail.com

Changed by



Python 3.
Library version:


"""

import vtk
import socket
import numpy as np
import time as tm
import sys

host = 'localhost'
port = 65432
if len(sys.argv) > 1:
    port = int(sys.argv[1])
printouts = True


class DroneHandle:
    def __init__(self, actor: vtk.vtkActor, host: str, port: int, printouts: bool):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.actor = actor
        self.printouts = printouts

        # setup socket
        self._open_socket(host, port)

    def animate(self, obj, event):
        try:
            data = self.socket.recv(1024)
            received = data.decode()
            if printouts: print("[INFO] Message received: ", received)
            if received == 'quit':
                self.socket.close()
                print("[INFO] Socket got closed")
                quit()
            else:
                roll, pitch, yaw = self._decode_message(message=received)
                self._update_vtk(obj,
                                 l_roll=self.roll, n_roll=roll,
                                 l_pitch=self.pitch, n_pitch=pitch,
                                 l_yaw=self.yaw, n_yaw=yaw)
                self._store_new_data(roll=roll, pitch=pitch, yaw=yaw)

        except OSError:
            # This is ugly but this should not happen in the real tests
            print("[INFO] Socket got closed")
            quit()

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
            quit()
        else:
            if self.printouts: print("[INFO] Connected")

    def _store_new_data(self, roll: float, pitch: float, yaw: float):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def _decode_message(self, message: str):
        meaning = message.split(" ")
        try:
            time, roll, pitch, yaw, x, y, z, t1, t2, t3, t4, windx, windy, windz = \
                [float(meaning[2 * i + 1]) for i in range(int(len(meaning) / 2))]
            return roll * 180/np.pi, pitch * 180/np.pi, yaw * 180/np.pi
        except ValueError:
            if self.printouts: print("[ERROR] Couldn't decode message")
            return 0, 0, 0

    def _update_vtk(self, obj, l_roll: float, l_pitch: float, l_yaw: float,
                    n_roll: float, n_pitch: float, n_yaw: float):
        # undo last rotation
        self.actor.RotateZ(-l_yaw)
        self.actor.RotateY(-l_pitch)
        self.actor.RotateX(-l_roll)
        # do new rotation
        self.actor.RotateX(n_roll)
        self.actor.RotateY(n_pitch)
        self.actor.RotateZ(n_yaw)
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