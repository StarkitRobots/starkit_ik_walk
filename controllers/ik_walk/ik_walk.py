

import numpy as np
import pandas as pd

from WalkController import WalkController

from setup.windows_setup import RobotSettingsWindow

from setup.robot_params import robot, params, modelParams
from setup.robot_params import send_command

controller = WalkController()

controlRanges = {
    "stepSizeX": (-0.1, 0.1),
    "stepSizeY": (-0.1, 0.1),
    "stepSizeYaw": (-15, 15)
}
name = "Control"
enableName = "enableWalk"

controlWindow = RobotSettingsWindow(controller.commands, controlRanges, name, enableName)

robotBody = robot.getFromDef('sahr')

phase = 0.0

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# pulseGen = RandomPulseGenerator(pulseDirection, pulseAmpl, pulseLength, 10.0, 5)
    
imu = robot.getDevice("imu")
imu.enable(timestep)

while robot.step(timestep) != -1:
    controlWindow.update()
    controller.step(timestep / 1000)
