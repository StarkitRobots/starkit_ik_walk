import numpy as np

from setup.robot_params import inputs

class StepTimingPipeline:

    COERCE = lambda self, x: x if x < 1 and x > -1 else 1 if x > 1 else -1

    def __init__(self, filter, smooth_dead):
        self.filter         = filter
        self.smooth_dead    = smooth_dead
        self.timing_weight  = 0

    def update_timing_weight(self, phase):
        self.timing_weight = self.COERCE(inputs.stepTiming.weightFactor * np.sin(np.pi * (phase * 2 - 1) - 0.5 * inputs.stepTiming.doubleSupportPhaseLen))
    
    def handle(self, value, phase):
        self.update_timing_weight(phase)
        timingFeed = self.smooth_dead.handle(self.filter.handle(value)) * self.timing_weight
        return (
            inputs.stepTiming.gainSpeedUp   * timingFeed[0] if timingFeed[0] >= 0.0 else 
            inputs.stepTiming.gainSlowDown  * timingFeed[0]
        )
