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

        # Roll Filters Length
        pFusedRollFilterLen:    int = 9
        dFusedRollFilterLen:    int = 30
        iFusedRollFilterLen:    int = 9

        # Pitch Filters Length
        pFusedPitchFilterLen:   int = 9
        dFusedPitchFilterLen:   int = 30
        iFusedPitchFilterLen:   int = 9

        # Roll Integrator Parameters
        iFusedRollFilterdT: float = 0.004
        iFusedRollFilterAlpha: float = (0.5) ** (8 / 0.004)

        # Pitch Integrator Paraneters
        iFusedPitchFilterdT:     float = 0.004 # 0.004, but need to be set using robot timestep
        iFusedPitchFilterAlpha:  float = 0.5

        # Roll Deadband Parameters
        pFusedRollDeadbandRadius: float = 0.1
        pFusedRollDeadbandCenter: float = 0.0
        dFusedRollDeadbandRadius: float = 0.2
        dFusedRollDeadbandCenter: float = 0.0
        
        #Pitch Deadband Parameters
        pFusedPitchDeadbandRadius: float = 0.025 #0.025 # 0.01
        pFusedPitchDeadbandCenter: float = 0.0
        dFusedPitchDeadbandRadius: float = 0.05 # 0.150 # 0.04
        dFusedPitchDeadbandCenter: float = 0.0

        # Roll Feedback Coefs (common for every stab move)
        pFusedRollFeedback: float = 1.0
        dFusedRollFeedback: float = 1.0
        iFusedRollFeedback: float = 1.0
        
        # Pitch Feedback Coefs (common for every stab move)
        pFusedPitchFeedback: float = 1.0
        dFusedPitchFeedback: float = 0.1 # 0.5 # 0.1
        iFusedPitchFeedback: float = 1.0

        # Roll Step Timing stabilization params
        enableStepTiming: bool = True

        stepTimingDeadbandRadius:   float = 0.02
        stepTimingDeadbandCenter:   float = 0.0

        gainSpeedUp:                float = 3.0
        gainSlowDown:               float = 5.3
        weightFactor:               float = 1.4
        
        doubleSupportPhaseLen:      float = 3.14
        gaitFrequencyMax:           float = 5.5

        # Roll COM Shift stabilization params
        enableFusedRollCOMShift: bool = False

        biasFusedRollCOMShift: float = 0.0
        pFusedRollFeedbackCOMShift: float = 0.2
        dFusedRollFeedbackCOMShift: float = 0.05
        iFusedRollFeedbackCOMShift: float = 0.0
        
        # Roll Hip stabilization params
        enableFusedRollHip: bool = False

        biasFusedRollHip: float = 0.0
        pFusedRollFeedbackHip: float = 3.0
        dFusedRollFeedbackHip: float = 2.0
        iFusedRollFeedbackHip: float = 0.0
        
        # Pitch Arm stabilization params
        enableFusedPitchArm: bool = False

        armExtension: float = 0.09
        
        biasFusedPitchArm:      float = 0.0
        pFusedPitchFeedbackArm: float = 0.2
        dFusedPitchFeedbackArm: float = -1.0   #-4.4 #-1.0
        iFusedPitchFeedbackArm: float = 0.0

        # Pitch Hip stabilization params
        enableFusedPitchHip: bool = False

        biasFusedPitchHip: float = 0.0
        pFusedPitchFeedbackHip: float = 0.75 # 0.4
        dFusedPitchFeedbackHip: float = -1.0 # 0.6
        iFusedPitchFeedbackHip: float = 0.0


        # expected angles params

        defaultFusedRoll:        float = 0.0
        defaultFusedPitch:       float = -0.150
        expectedAmplFusedRoll:   float = 0.0485
        expectedAmplFusedPitch:  float = 0.0
        expectedPhaseFusedRoll:  float = -np.pi - 0.6
        expectedPhaseFusedPitch: float = -np.pi - 0.6

    def __init__(self) -> None:
        self.params = self.Parameters()

        # Roll Filters Setup
        self.pFusedRollFilter   = MeanFilter(self.params.pFusedRollFilterLen)
        self.dFusedRollFilter   = WLBF_Filter(self.params.dFusedRollFilterLen)
        self.iFusedRollFilter   = MeanFilter(self.params.iFusedRollFilterLen)

        # Pitch Filters Setup
        self.pFusedPitchFilter  = MeanFilter(self.params.pFusedPitchFilterLen)
        self.dFusedPitchFilter  = WLBF_Filter(self.params.dFusedPitchFilterLen)
        self.iFusedPitchFilter  = MeanFilter(self.params.iFusedPitchFilterLen)

        # Integrators Setup
        self.iFusedRollIntegrator = EWIntegrator(self.params.iFusedRollFilterdT, self.params.iFusedRollFilterAlpha)
        # self.iFusedRollIntegrator.setHalfLifeAlpha(8 / 0.004)
        self.iFusedPitchIntegrator = EWIntegrator(self.params.iFusedPitchFilterdT, self.params.iFusedPitchFilterAlpha)
        # self.iFusedPitchIntegrator.setHalfLifeAlpha(8 / 0.004)

        # Roll Smooth Deadband Setup
        self.pFusedRollSmoothDeadband = SmoothDeadband(self.params.pFusedRollDeadbandRadius, self.params.pFusedRollDeadbandCenter)
        self.dFusedRollSmoothDeadband = SmoothDeadband(self.params.dFusedRollDeadbandRadius, self.params.dFusedRollDeadbandCenter)

        # Pitch Smooth Deadband Setup
        self.pFusedPitchSmoothDeadband   = SmoothDeadband(self.params.pFusedPitchDeadbandRadius, self.params.pFusedPitchDeadbandCenter)
        self.dFusedPitchSmoothDeadband   = SmoothDeadband(self.params.dFusedPitchDeadbandRadius, self.params.dFusedPitchDeadbandCenter)

        # Step Timing Setup
        self.stepTimingFilter = MeanFilter(self.params.pFusedRollFilterLen) # step timing filter is same as proportional fused roll
        self.stepTimingSmoothDeadband = SmoothDeadband(self.params.stepTimingDeadbandRadius, self.params.stepTimingDeadbandCenter)

        # Fused Roll Feedback storage
        self.pFusedRollFeedback = 0.0
        self.dFusedRollFeedback = 0.0
        self.iFusedRollFeedback = 0.0

        # Fused Pitch Feedback storage
        self.pFusedPitchFeedback = 0.0
        self.dFusedPitchFeedback = 0.0
        self.iFusedPitchFeedback = 0.0
        
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
    
    def updateFeedback(self, fusedRollDeviation: float, fusedPitchDeviation: float, elapsed: float) -> None:
        # Roll Smooth Deadband Setup
        self.pFusedRollSmoothDeadband = SmoothDeadband(self.params.pFusedRollDeadbandRadius, self.params.pFusedRollDeadbandCenter)
        self.dFusedRollSmoothDeadband = SmoothDeadband(self.params.dFusedRollDeadbandRadius, self.params.dFusedRollDeadbandCenter)

        # Pitch Smooth Deadband Setup
        self.pFusedPitchSmoothDeadband   = SmoothDeadband(self.params.pFusedPitchDeadbandRadius, self.params.pFusedPitchDeadbandCenter)
        self.dFusedPitchSmoothDeadband   = SmoothDeadband(self.params.dFusedPitchDeadbandRadius, self.params.dFusedPitchDeadbandCenter)

        self.pFusedRollFeedback = self.pFusedRollSmoothDeadband .handle(self.pFusedRollFilter       .handle(fusedRollDeviation))            * self.params.pFusedRollFeedback
        self.dFusedRollFeedback = self.dFusedRollSmoothDeadband .handle(self.dFusedRollFilter       .handle(fusedRollDeviation, elapsed))   * self.params.dFusedRollFeedback
        self.iFusedRollFeedback = self.iFusedRollFilter         .handle(self.iFusedRollIntegrator   .handle(fusedRollDeviation))            * self.params.iFusedRollFeedback


        self.pFusedPitchFeedback = self.pFusedPitchSmoothDeadband   .handle(self.pFusedPitchFilter      .handle(fusedPitchDeviation))              * self.params.pFusedPitchFeedback
        self.dFusedPitchFeedback = self.dFusedPitchSmoothDeadband   .handle(self.dFusedPitchFilter      .handle(fusedPitchDeviation, elapsed))     * self.params.dFusedPitchFeedback
        self.iFusedPitchFeedback = self.iFusedPitchFilter           .handle(self.iFusedPitchIntegrator  .handle(fusedPitchDeviation))              * self.params.iFusedPitchFeedback

    def stepTimingFeedback(self, value: float, phase: float) -> float:
        if not self.params.enableStepTiming:
            return 0.0

        timingWeight = np.clip(self.params.weightFactor * np.sin(np.pi * (phase * 2.0 - 1.0) - 0.5 * self.params.doubleSupportPhaseLen), -1, 1)
        timingFeedback = self.stepTimingSmoothDeadband.handle(self.stepTimingFilter.handle(value)) * timingWeight

        return (
            self.params.gainSpeedUp     * timingFeedback if timingFeedback >= 0.0 else
            self.params.gainSlowDown    * timingFeedback
        )
    
    def fusedRollCOMShiftFeedback(self) -> float:
        return (self.params.biasFusedRollCOMShift 
                + self.params.pFusedRollFeedbackCOMShift * self.pFusedRollFeedback
                + self.params.dFusedRollFeedbackCOMShift * self.dFusedRollFeedback
                + self.params.iFusedRollFeedbackCOMShift * self.iFusedRollFeedback) if self.params.enableFusedRollCOMShift else 0.0
    
    def fusedRollHipFeedback(self) -> float:
        return (self.params.biasFusedRollHip
                + self.params.pFusedRollFeedbackHip * self.pFusedRollFeedback
                + self.params.dFusedRollFeedbackHip * self.dFusedRollFeedback
                + self.params.iFusedRollFeedbackHip * self.iFusedRollFeedback) if self.params.enableFusedRollHip else 0.0
    
    def fusedPitchArmFeedback(self) -> float:
        return (self.params.biasFusedPitchArm 
                + self.params.pFusedPitchFeedbackArm * self.pFusedPitchFeedback 
                + self.params.dFusedPitchFeedbackArm * self.dFusedPitchFeedback
                + self.params.iFusedPitchFeedbackArm * self.iFusedPitchFeedback) if self.params.enableFusedPitchArm else 0.0
    
    def fusedPitchHipFeedback(self) -> float:
        print("Proportional Pitch Filtered", self.pFusedPitchFilter.mean())
        print("Proportional Pitch Feedback", self.pFusedPitchFeedback)
        print("Derivative Pitch Filtered", self.dFusedPitchFilter.deriv())
        print("Derivative Pitch Feedback", self.dFusedPitchFeedback)
        # print("Integral Pitch Feedback", self.iFusedPitchFeedback)

        return (self.params.biasFusedPitchHip
                + self.params.pFusedPitchFeedbackHip * self.pFusedPitchFeedback
                + self.params.dFusedPitchFeedbackHip * self.dFusedPitchFeedback
                + self.params.iFusedPitchFeedbackHip * self.iFusedPitchFeedback) if self.params.enableFusedPitchHip else self.params.biasFusedPitchHip
    

        