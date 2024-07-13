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

        self.pulsePoints = np.array([0.0] * count)
        self.currentPulse = 0

        self.time = 0.0

        self.launch()

    def timeToNextPulse(self) -> float:
        return self.pulsePoints[self.currentPulse] - self.time if self.currentPulse < len(self.pulsePoints) else 0.0

    def launch(self) -> None:
        self.pulsePoints = np.array([self.timeOverall / self.count * i for i in range(self.count)])
        self.pulsePoints += np.random.uniform(0.0, self.timeOverall / self.count - self.length, self.count)
        self.currentPulse = 0

        self.time = 0.0

    def relaunchRequired(self) -> bool:
        return self.time > self.timeOverall
    
    def isPulse(self) -> bool:
        if self.currentPulse < len(self.pulsePoints) and self.time >= self.pulsePoints[self.currentPulse] + self.length:
            self.currentPulse += 1

        return self.currentPulse < len(self.pulsePoints) and self.pulsePoints[self.currentPulse] <= self.time < self.pulsePoints[self.currentPulse] + self.length
    
class RandomDelayedPulseGenerator(RandomPulseGenerator):

    def __init__(self, direction: np.ndarray, ampl: float = None, length: float = 0.5, timeOverall: float = 10, count: int = 5, delay: float=0.0) -> None:
        assert length * count < timeOverall - delay * count

        self.__delay = delay

        super().__init__(direction, ampl, length, timeOverall, count)

    def launch(self) -> None:
        self.pulsePoints = np.array([self.timeOverall / self.count * i for i in range(self.count)])
        self.pulsePoints += np.random.uniform(self.__delay, self.timeOverall / self.count - self.length, self.count)
        self.currentPulse = 0

        self.time = 0.0