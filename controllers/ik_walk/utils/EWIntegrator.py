class EWIntegrator:

    def __init__(self, dT: float, alpha: float) -> None:
        self.__alpha = alpha
        self.__dT = dT
        self.__integral = 0

    def integrate(self, value: float) -> float:
        self.__integral = self.__dT * value + self.__alpha * self.__integral
        return self.__integral
    
    def handle(self, value: float) -> float:
        return self.integrate(value)
    
    def setHalfLifeAlpha(self, cycles: float) -> None:
        self.__alpha = 0.0 if cycles < 1e-6 else 0.5 ** (1.0 / cycles)