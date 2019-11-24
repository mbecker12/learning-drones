"""
created by Jan Schiffeler on 06.11.12019
python 3.6.4
numpy 1.16


"""

import numpy as np
import matplotlib.pyplot as plt


def torus(position: np.ndarray, size: int):
    """
    if a position exceeds the grid it reappears on the opposing side
    :param position:
    :param size:
    :return:
    """
    if position > size:
        position = 0
    elif position < 0:
        position = size
    return position


class Human:
    def __init__(self, size: int, diffusion_rate: float):
        self.size = size
        self.diffusionrate = diffusion_rate
        self.pos = np.random.randint(0, self.size+1, [1, 2])
        self.torus = np.vectorize(torus)

    def walk(self):
        """
        perform random walk
        :return:
        """
        r = np.random.rand()
        if 1 - self.diffusionrate < r:
            step = np.random.choice([-1, 1])
            self.pos[0, np.random.randint(0, 2)] += step
            self.pos = self.torus(self.pos, self.size)


if __name__ == "__main__":
    size = 10
    d = 0.9
    h = Human(size, d)
    fig, ax = plt.subplots()
    ax.set(title='random walk with d={}'.format(d))
    plt.ion()
    pos = np.array([[int(size / 2), int(size / 2)]])
    plt.xlim(0, size)
    plt.ylim(0, size)
    plt.grid()

    major_ticks = np.arange(0, size+1, 5)
    minor_ticks = np.arange(0, size+1, 1)

    ax.set_xticks(major_ticks)
    ax.set_xticks(minor_ticks, minor=True)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks, minor=True)

    ax.grid(which='minor', alpha=0.3)
    ax.grid(which='major', alpha=0.6)

    dot = ax.scatter(pos[0, 0], pos[0, 1], c='b')
    old = ax.scatter(pos[0, 0], pos[0, 1], c='r', alpha=0.5, s=5)
    old_pos = [pos]

    plt.draw()

    for i in range(100):
        h.walk()
        pos = h.pos

        arr = np.array(old_pos).transpose(1, 0, 2)[0, :, :]
        old.set_offsets(arr)

        dot.set_offsets(pos)

        fig.canvas.draw_idle()
        plt.pause(0.1)

        old_pos.append(pos)


