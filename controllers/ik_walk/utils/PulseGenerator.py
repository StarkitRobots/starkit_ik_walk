from abc import abstractmethod

import numpy as np

class PulseGeneratorBase:

    def __init__(self, direction: np.ndarray, ampl: float=None) -> None:
        assert np.linalg.norm(direction) != 0
        self.direction = direction / np.linalg.norm(direction)

        self.ampl = ampl if ampl is not None else np.linalg.norm(direction)

    @abstractmethod
    def pulse(self) -> np.ndarray:
        pass
    
class SinglePulseGenerator(PulseGeneratorBase):
    
    def pulse(self) -> np.ndarray:
        return self.direction * self.ampl
    
class TimedPulseGenerator(SinglePulseGenerator):

    def __init__(self, direction: np.ndarray, ampl: float = None) -> None:
        super().__init__(direction, ampl)

        self.time = 0.0

    def launch(self) -> None:
        self.time = 0.0

    def tick(self, elapsed: float) -> np.ndarray:
        self.time += elapsed

        if self.relaunchRequired():
            self.launch()

        return self.pulse() if self.isPulse() else self.direction * 0.0


    @abstractmethod
    def relaunchRequired(self) -> bool:
        pass

    @abstractmethod
    def isPulse(self) -> bool:
        pass


class PeriodicPulseGenerator(TimedPulseGenerator):

    def __init__(self, direction: np.ndarray, ampl: float = None, 
                 length: float=0.5, period: float=1.0, offset: float=0.0) -> None:
        assert period >= length + offset

        super().__init__(direction, ampl)
        
        self.__length = length
        self.__period = period
        self.__offset = offset

    def relaunchRequired(self) -> bool:
        return self.time > self.__period
    
    def isPulse(self) -> bool:
        return self.__offset <= self.time < self.__length + self.__offset
    
class RandomPulseGenerator(TimedPulseGenerator):

    def __init__(self, direction: np.ndarray, ampl: float = None, length: float=0.5, timeOverall: float=10.0, count: int=5) -> None:
        assert length * count < timeOverall

        super().__init__(direction, ampl)

        self.length = length
        self.timeOverall = timeOverall
        self.count = count

        self.__pulsePoints = np.array([0.0] * count)
        self.__currentPulse = 0

        self.time = 0.0

        self.launch()


    def launch(self) -> None:
        self.__pulsePoints = np.array([self.timeOverall / self.count * i for i in range(self.count)])
        self.__pulsePoints += np.random.uniform(0.0, self.timeOverall / self.count - self.length, self.count)
        self.__currentPulse = 0

        self.time = 0.0

    def relaunchRequired(self) -> bool:
        return self.time > self.timeOverall
    
    def isPulse(self) -> bool:
        if self.__currentPulse < len(self.__pulsePoints) and self.time >= self.__pulsePoints[self.__currentPulse] + self.length:
            self.__currentPulse += 1

        return self.__currentPulse < len(self.__pulsePoints) and self.__pulsePoints[self.__currentPulse] <= self.time < self.__pulsePoints[self.__currentPulse] + self.length