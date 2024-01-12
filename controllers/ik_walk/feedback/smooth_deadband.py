import numpy as np


class SmoothDeadband:

    def __init__(self, radius, center=0.0):
        self.radius = radius
        self.center = center

    def handle(self, value):
        return self(value)

    def __call__(self, value):
        x = value - self.center
        sign = np.sign(x)

        square_part = np.abs(x) <= 2 * self.radius
        return ((x - sign * self.radius) * (~square_part) + 
                (sign * x ** 2 / (4 * self.radius)) * square_part)