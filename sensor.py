import re
import numpy as np
import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# TODO: Rewrite Sensor class based on updated physical model 
class Sensor:
    """
    Very Simple Sensor class, able to:
    - obtain acceleration from outside
    - return current acceleration
    - time-integrate to obtain position and velocity

    TODO: figure out more robust ways to infere position
    and velocity from noisy data; i.e. find more accurate
    and reliable integration schemes.

    TODO: Research more deeply typical inputs and outputs of
    sensors (e.g. accelerometers), for instance: do they have
    memory of the last couple of acceleration values?
    """
    def __init__(self, delta_t, previous_pos, previous_vel):
        self.current_acceleration = 0.0
        self.last_acceleration = 0.0
        self.previous_pos = previous_pos
        self.previous_vel = previous_vel
        self.delta_t = delta_t

    def set_position(self, position):
        self.previous_pos = position

    def set_velocity(self, velocity):
        self.previous_vel = velocity

    def return_acceleration(self):
        return self.current_acceleration

    def measure_acceleration(self, acceleration):
        assert(isinstance(acceleration,
            (float, np.float16, np.float32, np.float64)))
        if self.last_acceleration == 0:
            logger.debug("Previous Acceleration Set to 0. Supposed to be done in Initialization only.")
        self.last_acceleration = self.current_acceleration
        self.current_acceleration = acceleration

    def naive_get_delta_v(self):
        delta_v = self.current_acceleration * self.delta_t
        return delta_v

    def naive_get_delta_x(self, velocity):
        delta_x = velocity * self.delta_t + \
            0.5 * self.current_acceleration * self.delta_t * self.delta_t
        return delta_x

    def verlet_get_delta_x(self, velocity):
        delta_x = velocity * self.delta_t + \
            0.5 * self.current_acceleration * self.delta_t * self.delta_t
        return delta_x

    def verlet_get_delta_v(self):
        delta_v = 0.5 * (self.last_acceleration + self.current_acceleration) * self.delta_t
        return delta_v

    def get_current_x_and_v(self, return_vel=False):
        velocity = self.previous_vel + self.naive_get_delta_v()
        position = self.previous_pos + self.naive_get_delta_x(velocity)
        self.previous_pos = position
        self.previous_vel = velocity
        if not return_vel:
            return position
        else:
            return position, velocity

    def velocity_verlet(self, return_vel=True):
        position = self.previous_pos + self.verlet_get_delta_x(self.previous_vel)
        velocity = self.previous_vel + self.verlet_get_delta_v()
        self.previous_pos = position
        self.previous_vel = velocity
        if not return_vel:
            return position
        else:
            return position, velocity


class DataGenerator:
    """
    Preliminary Data Generator class to simulate input data
    and time series for sensor simulation.
    """
    def __init__(
            self,
            noise=0.0,
            method=None,
            **kwargs):

        self.noise_level = 0.0
        self.method = method

        if re.match(r"[cC]onst", self.method):
            self.const = kwargs['const']

        elif re.match(r"[lL]inear", self.method):
            self.time = kwargs['time']
            self.slope = kwargs['slope']
            self.const = kwargs['const']
            self.timestep = kwargs['timestep']
        else:
            raise Exception("Method " + method + " Not Supported!")

    def __call__(self):
        if re.match(r"[cC]onst", self.method):
            while True:
                yield self.const
        elif re.match(r"[lL]inear", self.method):
            while True:
                self.time += self.timestep
                yield self.const + self.time * self.slope


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    plt_logger = logging.getLogger('matplotlib')
    plt_logger.setLevel(logging.INFO)

    fig, (ax0, ax1, ax2) = plt.subplots(1, 3)

    # ###### CASE 0
    # ###### Free Fall
    n = 101
    dt = 0.1
    time_grid = np.arange(0, n)

    x = np.zeros(n)
    v = np.zeros(n)
    a = np.zeros(n)
    acc = 9.81

    x[0] = 0
    v[0] = 0
    s = Sensor(dt)
    for t in time_grid:
        if t == n-1:
            break
        s.measure_acceleration(acc + np.random.normal(0.0, 0.0))
        a[t+1] = s.return_acceleration()
        x[t+1], v[t+1] = s.get_current_x_and_v(x[t], v[t])
        # x[t+1], v[t+1] = s.velocity_verlet(x[t], v[t])

    ax0.set_title("Free Fall, Naive")
    ax0.plot(a[1:], label='a')
    ax0.plot(v[1:], label='v')
    ax0.plot(x[1:], label='x')
    ax0.plot(dt * time_grid * acc + v[0], label='lin')
    ax0.plot(x[0] + v[0] * (dt * time_grid) + 0.5 * acc * (dt * time_grid)
        * (dt * time_grid), label='sq')
    ax0.legend()
    s = 0.5 * acc * (dt * time_grid) * (dt * time_grid) + x[0] + v[0] * (dt * time_grid)
    print("s: ", s[n-1])
    print("x: ", x[n-2])


    # ###### CASE 1
    # ###### Constant Acceleration
    n = 101
    dt = 0.1
    time_grid = np.arange(0, n)

    x = np.zeros(n)
    v = np.zeros(n)
    a = np.zeros(n)
    acc = -5

    v_old = 0
    x_old = 0
    x[0] = 10
    v[0] = 10
    s = Sensor(dt)
    for t in time_grid:
        if t == n-1:
            break
        s.measure_acceleration(acc + np.random.normal(0.0, 0.0))
        a[t+1] = s.return_acceleration()
        dv = s.naive_get_delta_v()
        # x[t+1], v[t+1] = s.get_current_x_and_v(x[t], v[t])
        x[t+1], v[t+1] = s.velocity_verlet(x[t], v[t])

    ax1.set_title("Constant acceleration, VV")
    ax1.plot(a[1:], label='a')
    ax1.plot(v[1:], label='v')
    ax1.plot(x[1:], label='x')
    ax1.plot(dt * time_grid * acc + v[0], label='lin')
    ax1.plot(x[0] + v[0] * (dt * time_grid) + 0.5 * acc * (dt * time_grid)
        * (dt * time_grid), label='sq')
    ax1.legend()
    s = 0.5 * acc * (dt * time_grid) * (dt * time_grid) + x[0] + v[0] * (dt * time_grid)
    print("s: ", s[n-1])
    print("x: ", x[n-2])


    ###### CASE 2
    ###### Periodic Acceleration
    n = 101
    dt = 0.1
    omega = 0.25
    time_grid = np.arange(0, n)

    x = np.zeros(n)
    v = np.zeros(n)
    a = np.zeros(n)
    v_old = 0
    x_old = 0

    s = Sensor(dt)
    for t in time_grid:
        if t == n-1:
            break
        s.measure_acceleration(np.sin(omega * time_grid[t] + np.random.normal(0.0, 0.0)))
        a[t+1] = s.return_acceleration()
        dv = s.naive_get_delta_v()
        # x[t+1], v[t+1] = s.get_current_x_and_v(x[t], v[t])
        x[t+1], v[t+1] = s.velocity_verlet(x[t], v[t])

    ax2.set_title("Periodic Acceleration, VV")
    ax2.plot(a[1:], label='a')
    ax2.plot(v[1:], label='v')
    ax2.plot(x[1:], label='x')
    
    ax2.plot(
        -dt / omega * np.cos(omega * time_grid) + dt / omega,
        label='v_theo')
    ax2.plot(dt * dt / omega * time_grid + 
        -dt * dt / (omega * omega) * np.sin(omega * time_grid), 
        label='x_theo')
    ax2.legend()
    plt.tight_layout()
    plt.show()
