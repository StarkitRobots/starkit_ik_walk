import numpy as np

class WLBFFilter:

    def __init__(self, size, default_dx=None) -> None:
        if default_dx is None:
            default_dx = np.zeros(size[1])

        self.x_buff = np.zeros(size)
        self.y_buff = np.zeros(size)
        self.w_buff = np.zeros(size)

        self.a = np.zeros(size[1])
        self.b = np.zeros(size[1])
        self.x_mean = np.zeros(size[1])

        self.default_dx = default_dx

    def deriv(self):
        return self.b

    def put(self, x, y, w=0):
        self.x_buff = np.append(self.x_buff[1:], [x], axis=0)
        self.y_buff = np.append(self.y_buff[1:], [y], axis=0)
        self.w_buff = np.append(self.w_buff[1:], [w], axis=0)

        return self
    
    def handle(self, value):
        self.put(self.x_buff[-1] + self.default_dx, value, np.ones(self.default_dx.size))
        return self.updatedDeriv()

    def updatedDeriv(self):
        self.calculate_wlbf()
        return self.deriv()
    
    def calculate_wlbf(self):
        xmean = np.sum(self.w_buff * self.x_buff, axis=0)
        ymean = np.sum(self.w_buff * self.y_buff, axis=0)
        wsum = np.sum(self.w_buff, axis=0)

        for i, ws in enumerate(wsum):
            if ws == 0:
                self.a[i] = self.b[i] = self.x_mean[i] = 0
            else:
                xmean[i] /= ws
                ymean[i] /= ws

        sumwx = np.sum(self.w_buff * (self.x_buff - xmean), axis=0)
        sumwy = np.sum(self.w_buff * (self.y_buff - ymean), axis=0)
        sumwxx = np.sum(self.w_buff * (self.x_buff - xmean) ** 2, axis=0)
        sumwxy = np.sum(self.w_buff * (self.x_buff - xmean) * (self.y_buff - ymean), axis=0)

        denominator = wsum * sumwxx - sumwx ** 2

        for i, d in enumerate(denominator):
            if d == 0:
                self.a[i] = ymean[i]
                self.b[i] = 0
            else:
                self.a[i] = (sumwy[i] * sumwxx[i] - sumwxy[i] * sumwx[i]) / d
                self.b[i] = (wsum[i] * sumwxy[i] - sumwx[i] * sumwy[i]) / d
        
        self.x_mean = xmean
        return