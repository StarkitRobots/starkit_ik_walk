import numpy as np

from utils.PulseGenerator import RandomPulseGenerator
from utils.PulseGenerator import RandomDelayedPulseGenerator

enableRandomPulse = True

pulseGen = None
pulseLabels = dict()
pulseRanges = dict()

if enableRandomPulse:
    pulseDirection = np.array([1.0, 0.0, 0.0])
    pulseAmpl = 12.5    # N
    pulseLength = 0.2   # sec
    timeOverall = 10.0  # sec
    count = 5
    delay = 1.0

    pulseGen = RandomDelayedPulseGenerator(pulseDirection, pulseAmpl, pulseLength, timeOverall, count, delay)

    pulseRanges = {
        "ampl": (0.0, 20.0),
        "length": (0.0, 2.0),
        "timeOverall": (0.0, 100.0),
        "count": (0, 20),
        # "delay": (0.0, 1.0)
    }
