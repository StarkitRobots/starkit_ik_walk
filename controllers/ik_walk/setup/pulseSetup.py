import numpy as np

from utils.PulseGenerator import RandomPulseGenerator

enableRandomPulse = True

pulseGen = None
pulseLabels = dict()
pulseRanges = dict()

if enableRandomPulse:
    pulseDirection = np.array([0.0, 1.0, 0.0])
    pulseAmpl = 1.0     # N
    pulseLength = 0.2   # sec
    timeOverall = 10.0  # sec
    count = 5

    pulseGen = RandomPulseGenerator(pulseDirection, pulseAmpl, pulseLength, timeOverall, count)

    pulseRanges = {
        "ampl": (0.0, 20.0),
        "length": (0.0, 2.0),
        "timeOverall": (0.0, 100.0),
        "count": (0, 20)
    }
