class EWIntegrator:

    def __init__(self, dT, alpha):
        self.alpha = alpha
        self.dT = dT
        self.integral = 0

    def integrate(self, value):
        self.integral = self.dT * value + self.alpha * self.integral
        return self.integral
    
    def handle(self, value):
        return self.integrate(value)   
    
    def set_half_life_alpha(self, cycles):
        self.alpha = 0.0 if cycles < 1e-6 else 0.5 ** (1.0 / cycles)