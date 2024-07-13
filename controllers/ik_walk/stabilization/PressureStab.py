from dataclasses import dataclass

import numpy as np

class PressureStab:

    @dataclass
    class Parameters:
        leftSensorsCenter:  np.ndarray = np.array([0, 1])
        leftSensorsEdge:    np.ndarray = np.array([2, 3])

        rightSensorsCenter: np.ndarray = np.array([2, 3])
        rightSensorsEdge:   np.ndarray = np.array([0, 1])

        centerSensorsWeightY:   float = 0.9
        edgeSensorsWeightY:     float = 2.4

        securityPhase:              float = 1.0
        securityPressureCenterY:    float = -0.05

    def __init__(self, frequency: float) -> None:
        self.params = self.Parameters()

        self.isWalking = False
        self.isLeftSupport = True
        self.stepDuration = 1.0 / (2.0 * frequency)
        self.timeSinceLastStep = 0.0

    def startWalking(self):
        self.isWalking = True
    
    def stopWalking(self):
        self.isWalking = False

    def getPressureCenterY(self, leftPressure: np.ndarray, rightPressure: np.ndarray) -> float:
        return (
            (np.sum(leftPressure[self.params.leftSensorsCenter]) * self.params.centerSensorsWeightY 
            + np.sum(leftPressure[self.params.leftSensorsEdge]) * self.params.edgeSensorsWeightY) 
            * (-1.0 if self.isLeftSupport else 1.0)
            + (np.sum(rightPressure[self.params.rightSensorsCenter]) * self.params.centerSensorsWeightY
            + np.sum(rightPressure[self.params.rightSensorsEdge]) * self.params.edgeSensorsWeightY)
            * (-1.0 if not self.isLeftSupport else 1.0)
        ) / (np.sum(leftPressure) + np.sum(rightPressure))
    
    def getCurrentPhase(self, leftPressure: np.ndarray, rightPressure: np.ndarray, elapsed: float) -> float:
        pressureCenterY = self.getPressureCenterY(leftPressure, rightPressure)

        walkProgressionMultiplier = 1.0

        prevStepPhase = self.timeSinceLastStep / self.stepDuration
        stepPhase = (self.timeSinceLastStep + elapsed) / self.stepDuration

        if (pressureCenterY < self.params.securityPressureCenterY 
            and (prevStepPhase < self.params.securityPhase and stepPhase >= self.params.securityPhase)):
            walkProgressionMultiplier = 0.0

        if self.isWalking:
            self.timeSinceLastStep += elapsed * walkProgressionMultiplier

        if self.timeSinceLastStep > self.stepDuration:
            self.timeSinceLastStep -= self.stepDuration
            self.isLeftSupport = not self.isLeftSupport

        return self.timeSinceLastStep / (2 * self.stepDuration) + 0.5 * (1 if not self.isLeftSupport else 0)

    
