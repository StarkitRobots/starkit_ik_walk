import numpy as np

class Expectation:

    DEFAULT_ANGLES = np.array([0.0, -0.150])

    EXPECTED_AMPLITUDE  = np.array([      0.0485,          0.0])
    EXPECTED_PHASE      = np.array([-np.pi - 0.6, -np.pi - 0.6])

    def __init__(self, phase):
        self.phase = phase

    def angles(self):
        return self.DEFAULT_ANGLES + self.EXPECTED_AMPLITUDE * np.sin((2* self.phase - 1) * np.pi - self.EXPECTED_PHASE)