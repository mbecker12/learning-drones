import re
import numpy as np
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


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
    def __init__(self, delta_t):
        self.acceleration = 0.0
        self.delta_t = delta_t

    def get_acceleration(self):
        return self.acceleration

    def measure_acceleration(self, acceleration):
        assert(isinstance(acceleration, (float, np.float16, np.float32, np.float64)))
        self.acceleration = acceleration

    def naive_get_delta_v(self):
        return self.acceleration * self.delta_t

    def naive_get_delta_x(self, velocity):
        return velocity * self.delta_t + \
            self.acceleration * self.delta_t * self.delta_t

    def get_current_x_and_v(self, previous_position, previous_velocity):
        velocity = previous_velocity + self.naive_get_delta_v()
        position = previous_position + self.naive_get_delta_x(velocity)
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
    fig, (ax1, ax2) = plt.subplots(1, 2)
    # ###### CASE 1
    # ###### Constant Acceleration
    n = 50
    dt = 0.1
    time_grid = np.arange(0, n)

    x = np.zeros(n)
    v = np.zeros(n)
    a = np.zeros(n)

    v_old = 0
    x_old = 0
    for t in time_grid:
        if t == n-1:
            break
        s = Sensor(dt)
        s.measure_acceleration(1. + np.random.normal(0.0, 0.0))
        a[t+1] = s.get_acceleration()
        dv = s.naive_get_delta_v()
        x[t+1], v[t+1] = s.get_current_x_and_v(x[t], v[t])

    # ax1.title("Constant acceleration")
    ax1.plot(a[1:], label='a')
    ax1.plot(v[1:], label='v')
    ax1.plot(x[1:], label='x')
    ax1.plot(dt * np.linspace(1, 50, 50), label='lin')
    ax1.plot(dt * dt * np.linspace(1, 50, 50)**2, label='sq')
    ax1.legend()


    ###### CASE 2
    ###### Periodic Acceleration
    n = 100
    dt = 0.1
    omega = 0.25
    time_grid = np.arange(0, n)

    x = np.zeros(n)
    v = np.zeros(n)
    a = np.zeros(n)
    v_old = 0
    x_old = 0

    for t in time_grid:
        if t == n-1:
            break
        s = Sensor(dt)
        s.measure_acceleration(np.sin(omega * time_grid[t] + np.random.normal(0.0, 0.0)))
        a[t+1] = s.get_acceleration()
        dv = s.naive_get_delta_v()
        x[t+1], v[t+1] = s.get_current_x_and_v(x[t], v[t])

    # ax2.title("Periodic Acceleration")
    ax2.plot(a[1:], label='a')
    ax2.plot(v[1:], label='v')
    ax2.plot(x[1:], label='x')
    ratio = -dt * dt / omega / omega * np.sin(omega * time_grid) + 0.4 * time_grid / x
    print('ratio: ', ratio)
    ax2.plot(-dt / omega * np.cos(omega * time_grid) + 0.4, label='v_theo')
    plt.plot(-dt / omega / omega * np.sin(omega * time_grid) + 0.4 * time_grid,
        label='x_theo')
    ax2.legend()
    plt.tight_layout()
    plt.show()
