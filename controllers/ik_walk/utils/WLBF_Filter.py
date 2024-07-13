import numpy as np

class WLBF_Filter:

    def __init__(self, size: float, default_dx=None) -> None:
        if default_dx is None:
            default_dx = 0.0

        self.__x_buff = np.zeros(size)
        self.__y_buff = np.zeros(size)
        self.__w_buff = np.zeros(size)

        self.__a = 0.0
        self.__b = 0.0
        self.__x_mean = np.zeros(size)

        self.__default_dx = default_dx

    def deriv(self) -> float:
        return self.__b

    def put(self, x: float, y: float, w: float=0.0) -> None:
        self.__x_buff = np.append(self.__x_buff[1:], x)
        self.__y_buff = np.append(self.__y_buff[1:], y)
        self.__w_buff = np.append(self.__w_buff[1:], w)
    
    def handle(self, value: float, elapsed: float) -> float:
        self.put(self.__x_buff[-1] + elapsed, value, 1)
        return self.updatedDeriv()

    def updatedDeriv(self) -> float:
        self.calculateWLBF()
        return self.deriv()
    
    def calculateWLBF(self) -> None:
        xmean = np.sum(self.__w_buff * self.__x_buff)
        ymean = np.sum(self.__w_buff * self.__y_buff)
        wsum = np.sum(self.__w_buff)

        # print("Derivative value buffer", self.__y_buff)

        if wsum == 0:
            self.__a = self.__b = self.__x_mean = 0
        else:
            xmean /= wsum
            ymean /= wsum

        sumwx   = np.sum(self.__w_buff * (self.__x_buff - xmean))
        sumwy   = np.sum(self.__w_buff * (self.__y_buff - ymean))
        sumwxx  = np.sum(self.__w_buff * (self.__x_buff - xmean) ** 2)
        sumwxy  = np.sum(self.__w_buff * (self.__x_buff - xmean) * (self.__y_buff - ymean))

        denominator = wsum * sumwxx - sumwx ** 2

        if denominator == 0:
            self.__a = ymean
            self.__b = 0.0
        else:
            self.__a = (sumwy * sumwxx - sumwxy * sumwx)  / denominator
            self.__b = (wsum * sumwxy - sumwx * sumwy)    / denominator
        
        self.__x_mean = xmean
        
        return
