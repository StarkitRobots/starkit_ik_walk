import numpy as np

class MeanFilter:

    def __init__(self, size):
        self.buffer = np.zeros(size)
        self.mean = np.zeros(size[1])

    def update(self, value):
        self.put(value)
        self.calculate_mean()
        return self.mean
    
    def handle(self, value):
        return self.update(value)

    def put(self, value):
        self.buffer = np.append(self.buffer[1:], [value], axis=0)

    def calculate_mean(self):
        self.mean = self.buffer.mean(axis=0)
    