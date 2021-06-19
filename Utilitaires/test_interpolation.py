import unittest
from interpolation import Interpolation
import matplotlib.pyplot as plt

class TestInterpolation(unittest.TestCase):

    def test_interpolation(self):
        x = [1.0, 2.5, 5]
        y = [2, 4, 6]
        new_x = [0.5*i for i in range(11)]
        Interpolation().set_points(x=x, y=y)
        new_y = [Interpolation().interpolation_output(t) for t in new_x]
        plt.plot(x, y)
        plt.plot(new_x, new_y)
        plt.show()
        self.assertAlmostEqual(new_y[0], 0)

if __name__ == '__main__':
    unittest.main()
