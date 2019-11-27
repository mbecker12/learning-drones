import re
import numpy as np
import logging
logging.basicConfig(level=logging.DEBUG)
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
