import numpy as np


class Coin:
    def __init__(self, position, radius, value=100):
        """
        :param position: shape (3, 1) array for position
        """
        assert position.shape == (3, 1), position.shape
        self.position = position
        self.x = position[0, 0]
        self.y = position[1, 0]
        self.z = position[2, 0]
        self.radius = radius
        self.value = value

    def is_in_vicinity(self, drone_position):
        assert drone_position.shape == (3, 1), drone_position.shape
        distance = np.linalg.norm(self.position - drone_position)

        return distance < self.radius

    def distance_drone_to_coin(self, drone_position):
        distance = np.linalg.norm(self.position - drone_position)
        return distance
