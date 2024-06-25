from dataclasses import dataclass

import numpy as np

from utils.EWIntegrator     import EWIntegrator
from utils.MeanFilter       import MeanFilter
from utils.SmoothDeadband   import SmoothDeadband
from utils.WLBF_Filter      import WLBF_Filter

class PIDStabilization:
    '''Provides partial nimbro closed-loop gait stabilization'''

    @dataclass
    class Parameters:
        # filter params
        pFusedRollFilterLen:    int = 9

        pFusedPitchFilterLen:   int = 9
        dFusedPitchFilterLen:   int = 30
        iFusedPitchFilterLen:   int = 9

        iFusedPitchFilterdT:     float = 4.0 # 0.004, but need to be set using robot timestep
        iFusedPitchFilterAlpha:  float = 0.5

        pFusedPitchDeadbandRadius: float = 0.01
        dFusedPitchDeadbandRadius: float = 0.04

        pFusedPicthFeedback: float = 1.0
        dFusedPitchFeedback: float = 0.1
        iFusedPitchFeedback: float = 1.0

        # arm stabilization params
        armExtension: float = 0.09
        
        biasFusedPitchArm:      float = 0.0
        pFusedPitchFeedbackArm: float = 0.2
        dFusedPitchFeedbackArm: float = 2.4
        iFusedPitchFeedbackArm: float = 0.0
        
        # step timing stabilization params
        stepTimingDeadbandRadius:   float = 0.02
        gainSpeedUp:                float = 5.0
        gainSlowDown:               float = 10.3
        weightFactor:               float = 1.4
        doubleSupportPhaseLen:      float = 3.14
        gaitFrequencyMax:           float = 5.5

        # expected angles params
        defaultFusedRoll:        float = 0.0
        defaultFusedPitch:       float = 0.0
        expectedAmplFusedRoll:   float = 0.0485
        expectedAmplFusedPitch:  float = 0.0
        expectedPhaseFusedRoll:  float = -np.pi - 0.6
        expectedPhaseFusedPitch: float = -np.pi - 0.6

    def __init__(self) -> None:
        self.params = self.Parameters()

        self.pFusedRollFilter   = MeanFilter(self.params.pFusedRollFilterLen)

        self.pFusedPitchFilter  = MeanFilter(self.params.pFusedPitchFilterLen)
        self.dFusedPitchFilter  = WLBF_Filter(self.params.dFusedPitchFilterLen)
        self.iFusedPitchFilter  = MeanFilter(self.params.iFusedPitchFilterLen)

        self.iFusedPitchIntegrator = EWIntegrator(self.params.iFusedPitchFilterdT, self.params.iFusedPitchFilterAlpha)

        self.pFusedPitchSmoothDeadband   = SmoothDeadband(self.params.pFusedPitchDeadbandRadius, 0.0)
        self.dFusedPitchSmoothDeadband   = SmoothDeadband(self.params.dFusedPitchDeadbandRadius, 0.0)

        self.stepTimingSmoothDeadband = SmoothDeadband(self.params.stepTimingDeadbandRadius, 0.0)
        
        self.phase = 0.0

    # TODO: expected angles may be in other class
    def expectedFusedRoll(self, phase: float) -> float:
        return (self.params.defaultFusedRoll 
                + self.params.expectedAmplFusedRoll 
                    * np.sin((2 * phase - 1) * np.pi - self.params.expectedPhaseFusedRoll))
    
    def expectedFusedPitch(self, phase: float) -> float:
        return (self.params.defaultFusedPitch
                + self.params.expectedAmplFusedPitch
                    * np.sin((2 * phase - 1) * np.pi - self.params.expectedPhaseFusedPitch))

    def stepTimingFeedback(self, value: float, phase: float) -> float:
        timingWeight = np.clip(self.params.weightFactor * np.sin(np.pi * (phase * 2.0 - 1.0) - 0.5 * self.params.doubleSupportPhaseLen), -1, 1)
        timingFeedback = self.stepTimingSmoothDeadband.handle(self.pFusedRollFilter.handle(value)) * timingWeight

        return (
            self.params.gainSpeedUp     * timingFeedback if timingFeedback >= 0.0 else
            self.params.gainSlowDown    * timingFeedback
        )
    
    def armFeedback(self, value: float, elapsed: float) -> float:
        pFusedPitchFeedback = self.pFusedPitchSmoothDeadband.handle(self.pFusedPitchFilter.handle(value))           * self.params.pFusedPicthFeedback
        dFusedPitchFeedback = self.dFusedPitchSmoothDeadband.handle(self.dFusedPitchFilter.handle(elapsed, value))  * self.params.dFusedPitchFeedback
        iFusedPitchFeedback = self.iFusedPitchFilter.handle(self.iFusedPitchIntegrator.handle(value))               * self.params.iFusedPitchFeedback

        return (self.params.biasFusedPitchArm 
                + self.params.pFusedPitchFeedbackArm * pFusedPitchFeedback 
                + self.params.dFusedPitchFeedbackArm * dFusedPitchFeedback
                + self.params.iFusedPitchFeedbackArm * iFusedPitchFeedback)
