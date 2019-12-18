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
    def __init__(self, actor: vtk.vtkActor, target: vtk.vtkActor, host: str, port: int, printouts: bool):

        # rotations
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.r_target = 0
        self.p_target = 0
        self.y_target = 0

        #translations
        self.x = 0
        self.y = 0
        self.z = 0
        self.x_target = 0
        self.y_target = 0
        self.z_target = 0
        self.actor = actor
        self.target = target
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
                message_type, roll, pitch, yaw, x, y, z = self._decode_message(message=received)
                if message_type:
                    self._update_vtk(obj, act=self.actor,
                                     l_roll=self.roll, n_roll=roll,
                                     l_pitch=self.pitch, n_pitch=pitch,
                                     l_yaw=self.yaw, n_yaw=yaw, x=x, y=y, z=z)
                    self._store_new_data(store=True, roll=roll, pitch=pitch, yaw=yaw, x=x, y=y, z=z)
                else:
                    self._update_vtk(obj, act=self.target,
                                     l_roll=self.r_target, n_roll=roll,
                                     l_pitch=self.p_target, n_pitch=pitch,
                                     l_yaw=self.y_target, n_yaw=yaw, x=x, y=y, z=z)
                    self._store_new_data(store=False, roll=roll, pitch=pitch, yaw=yaw, x=x, y=y, z=z)

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

    def _store_new_data(self, store: bool, roll: float, pitch: float, yaw: float, x: float, y: float, z: float):
        if store:
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw
            self.x = x
            self.y = y
            self.z = z
        else:
            self.r_target = roll
            self.p_target = pitch
            self.y_target = yaw
            self.x_target = x
            self.y_target = y
            self.z_target = z

    def _decode_message(self, message: str):
        meaning = message.split(" ")
        if meaning[0] == "SETPOINTS":
            try:
                roll, pitch, yaw, x, y, z = [float(meaning[i]) for i in range(2, len(meaning), 2)]
                return False, roll * 180/np.pi, pitch * 180/np.pi, yaw * 180/np.pi, x, y, z
            except ValueError:
                if self.printouts: print("[ERROR] Couldn't decode message")
                return True, self.r_target, self.p_target, self.y_target, self.x_target, self.y_target, self.z_target
        else:
            try:
                time, roll, pitch, yaw, x, y, z, t1, t2, t3, t4, windx, windy, windz = \
                    [float(meaning[2 * i + 1]) for i in range(int(len(meaning) / 2))]
                return True, roll * 180/np.pi, pitch * 180/np.pi, yaw * 180/np.pi, x, y, z
            except ValueError:
                if self.printouts: print("[ERROR] Couldn't decode message")
                return True, self.roll, self.pitch, self.yaw, self.x, self.y, self.z

    def _update_vtk(self, obj, act: vtk.vtkActor, l_roll: float, l_pitch: float, l_yaw: float,
                    n_roll: float, n_pitch: float, n_yaw: float, x: float, y: float, z: float):
        # undo last rotation
        act.RotateZ(-l_yaw)
        act.RotateY(-l_pitch)
        act.RotateX(-l_roll)
        # do new rotation
        act.RotateX(n_roll)
        act.RotateY(n_pitch)
        act.RotateZ(n_yaw)

        act.SetPosition(x, y, z)
        obj.GetRenderWindow().Render()


# create a rendering window and renderer
ren = vtk.vtkRenderer()
ren.SetBackground(1.0, 1.0, 1.0)

renWin = vtk.vtkRenderWindow()
renWin.SetSize(750, 750)
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

# target
targetActor = vtk.vtkActor()
targetActor.SetMapper(droneMapper)
targetActor.GetProperty().SetColor(0.0, 0.0, 1.0)
targetActor.GetProperty().SetOpacity(0.2)

# camera
camera = vtk.vtkCamera()
camera.SetPosition(200, 200, 200)
camera.SetRoll(-120)
camera.SetFocalPoint(0, 0, 0)

# assign actors to the renderer
ren.AddActor(droneActor)
ren.AddActor(targetActor)
ren.AddActor(axes)
ren.SetActiveCamera(camera)

# enable user interface interactor
renWin.Render()

iren.Initialize()

dh = DroneHandle(actor=droneActor, target=targetActor, host=host, port=port, printouts=printouts)

iren.AddObserver('TimerEvent', dh.animate)
iren.CreateRepeatingTimer(0)

iren.Start()