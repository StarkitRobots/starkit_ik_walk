from stabilization.PIDStab      import PIDStabilization
from stabilization.PressureStab import PressureStab

PIDStab = PIDStabilization()
pressStab = PressureStab(1.5)

fusedRollStabRanges = {
    "pFusedRollDeadbandRadius": (0.0, 1.0),
    "dFusedRollDeadbandRadius": (0.0, 1.0),
    "pFusedRollFeedback": (0.0, 1.0),
    "dFusedRollFeedback": (0.0, 1.0),
    "iFusedRollFeedback": (0.0, 1.0),
}

fusedPitchStabRanges = {
    "pFusedPitchDeadbandRadius": (0.0, 1.0),
    "dFusedPitchDeadbandRadius": (0.0, 1.0),
    "pFusedPitchFeedback": (0.0, 1.0),
    "dFusedPitchFeedback": (0.0, 1.0),
    "iFusedPitchFeedback": (0.0, 1.0),
}

enableStepTimingName = "enableStepTiming"
stepTimingRanges = {
    "stepTimingDeadbandRadius": (0.0, 1.0),
    "gainSpeedUp": (0.0, 20.0),
    "gainSlowDown": (0.0, 20.0),
    "weightFactor": (0.0, 10.0),
}

enableFusedRollCOMShift = "enableFusedRollCOMShift"
fusedRollCOMShiftRanges = {
    "biasFusedRollCOMShift": (-1.0, 1.0),
    "pFusedRollFeedbackCOMShift": (0.0, 5.0),
    "dFusedRollFeedbackCOMShift": (0.0, 1.0),
    "iFusedRollFeedbackCOMShift": (0.0, 5.0),
}

enableFusedRollHip = "enableFusedRollHip"
fusedRollHipRanges = {
    "biasFusedRollHip": (-1.0, 1.0),
    "pFusedRollFeedbackHip": (0.0, 5.0),
    "dFusedRollFeedbackHip": (0.0, 5.0),
    "iFusedRollFeedbackHip": (0.0, 5.0),
}

enableFusedPitchArm = "enableFusedPitchArm"
fusedPitchArmRanges = {
    "armExtension": (0.0, 1.0),
    "biasFusedPitchArm": (-1.0, 1.0),
    "pFusedPitchFeedbackArm": (0.0, 1.0),
    "dFusedPitchFeedbackArm": (-5.0, 5.0),
    "iFusedPitchFeedbackArm": (0.0, 5.0),
}

enableFusedPitchHip = "enableFusedPitchHip"
fusedPitchHipRanges = {
    "biasFusedPitchHip": (-1.0, 1.0),
    "pFusedPitchFeedbackHip": (0.0, 20.0),
    "dFusedPitchFeedbackHip": (-5.0, 5.0),
    "iFusedPitchFeedbackHip": (-1.0, 1.0)
}

pressStabLabels = dict()

pressStabRanges = {
    "centerSensorsWeightY": (0.0, 5.0),
    "edgeSensorsWeightY": (0.0, 5.0),
    "securityPressureCenterY": (-1.0, 1.0)
}
