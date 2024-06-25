import numpy as np

class MeanFilter:

    def __init__(self, size: float) -> None:
        self.__buffer = np.zeros(size)
        self.__mean = 0.0

    def update(self, value):
        self.put(value)
        self.calculateMean()
        return self.__mean
    
    def handle(self, value):
        return self.update(value)

    def put(self, value):
        self.__buffer = np.append(self.__buffer[1:], value)

    def calculateMean(self):
        self.__mean = self.__buffer.mean()
