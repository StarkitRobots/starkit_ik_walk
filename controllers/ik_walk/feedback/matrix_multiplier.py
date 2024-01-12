import numpy as np

class MatrixMultiplier:

    def __init__(self, matrix):
        self.matrix = matrix

    def multiply(self, value):
        return np.matmul(self.matrix, value)
    
    def handle(self, value):
        return self.multiply(value)