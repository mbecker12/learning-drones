import sys
import numpy as np
import numpy.testing as npt
import unittest
sys.path.append('./py')
from sensor import Sensor, DataGenerator


class TestSensor(unittest.TestCase):
    def test_sensor_base_functions(self):
        s = Sensor(0.1)
        s.measure_acceleration(3)
        a = s.get_acceleration()
        npt.assert_almost_equal(a, 3)

        x0 = 0
        v0 = 0
        x1, v1 = s.get_current_x_and_v(x0, v0)
        npt.assert_almost_equal(v1, 0.3)
        npt.assert_almost_equal(
            x1,
            0.3 * 0.1 + 3 * 0.1 * 0.1
        )

        x2, v2 = s.get_current_x_and_v(x1, v1)
        npt.assert_almost_equal(v2, 0.6)
        npt.assert_almost_equal(
            x2,
            x1 + v2 * 0.1 + 3 * 0.1 * 0.1
        )


class TestDataGenerator(unittest.TestCase):
    def test_data_generator(self):
        d1 = DataGenerator(method="const", const=1.0)
        for i in range(10):
            print(next(d1()))

        d2 = DataGenerator(method="linear", const=4.0, slope=0.5, timestep=0.2, time=0.0)
        for i in range(10):
            print(next(d2()))

        with self.assertRaises(Exception):
            d3 = DataGenerator(method="new")


if __name__ == "__main__":
    unittest.main()
