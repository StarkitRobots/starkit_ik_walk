import numpy as np

class SmoothDeadband:

    def __init__(self, radius: float, center:float=0.0) -> None:
        self.__radius = radius
        self.__center = center

    def handle(self, value: float) -> float:
        return self(value)

    def __call__(self, value: float) -> float:
        x = value - self.__center
        sign = np.sign(x)

        if np.abs(x) > 2.0 * self.__radius:
            return x - sign * self.__radius
        
        return sign * x ** 2 / (4.0 * self.__radius)
