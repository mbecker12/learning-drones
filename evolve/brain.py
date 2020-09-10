import numpy as np
import numpy.random as npr
import sys

sys.path.append("/home/marvin/Projects/Drones")
from drone.onboard_computer import ControlUnit

EPSILON = 1e-8


class NeuralNetwork(ControlUnit):
    def __init__(self, layer_spec={1: 32, 2: 16, 3: 8, 4: 4}, n_inputs=15):
        """
        layer_spec: dict of network layers
            <layer_number>: <number of neurons>
        """
        self.weights = []
        self.batch_norm_params = []
        self.n_layers = len(layer_spec)
        layer_spec[0] = n_inputs
        self.layer_spec = layer_spec
        for layer_num, n_neurons in layer_spec.items():
            if layer_num == 0:
                continue
            layer_inputs = layer_spec[layer_num - 1]

            self.weights.append(
                npr.normal(
                    0.0, 1.0 / np.sqrt(n_neurons), size=(n_neurons, layer_inputs + 1)
                )
            )

    def _relu(self, x, thresh=0, alpha=1.0):
        if x < thresh:
            return 0.0
        else:
            return alpha * x

    def relu(self, x_vec, thresh=0, alpha=1.0):
        return np.array(
            [[self._relu(x, thresh=thresh, alpha=alpha) for x in np.squeeze(x_vec)]]
        ).T

    def sigmoid(self, x, beta=0.4):
        out = 1 / (np.exp(-beta * x) + 1)
        return out

    def feed_forward(self, input_vec):
        # TODO
        # maybe add normalization and batch_norm
        # what would be a sensible normalization for distance and velocity measures
        x = np.tanh(input_vec * 0.1)
        # print(x)

        for i in range(0, self.n_layers):
            last_layer = i == (self.n_layers - 1)
            x = np.vstack((x, [[1]]))
            x = np.dot(self.weights[i], x)

            assert x.shape[1] == 1

            if not last_layer:
                x = self.relu(x)
            else:
                x = self.sigmoid(x)

        return x

    def translate_input_to_thrust(
        self,
        drone_position,
        drone_velocity,
        drone_angle,
        drone_angle_velocity,
        coin_position,
    ):

        network_inputs = np.vstack(
            (
                drone_position,
                drone_velocity,
                drone_angle,
                drone_angle_velocity,
                coin_position,
            )
        )

        thrust = self.feed_forward(network_inputs)
        return thrust


if __name__ == "__main__":
    nn = NeuralNetwork()
    pos = np.zeros((3, 1))
    coin_pos = np.array([[10], [10], [10]])

    thrust = nn.translate_input_to_thrust(pos, pos, pos, pos, coin_pos)
    print(f"thrust: {thrust}")
