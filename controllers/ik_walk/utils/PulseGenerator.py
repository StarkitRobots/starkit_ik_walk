import numpy as np

class PulseGenerator:

    def __init__(self, direction: np.ndarray, period: float, length: float, ampl: float=None, offset: float=0.0) -> None:
        self.direction = direction / np.linalg.norm(direction)

        self.ampl = ampl if ampl is not None else np.linalg.norm(direction)

        assert period >= length + offset

        self.length = length
        self.period = period
        self.offset = offset

        self.time = 0.0

    def pulse(self, elapsed: float) -> np.ndarray:
        self.time += elapsed
        self.time -= self.period if self.time > self.period else 0.0

        return self.ampl * self.direction if self.offset <= self.time < self.length + self.offset else 0.0 * self.direction
        