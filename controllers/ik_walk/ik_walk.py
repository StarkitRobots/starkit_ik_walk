from sys import stderr
from pathlib import Path

import numpy as np
import pandas as pd

import starkit_ik_walk as sk

import setup.setup

from setup.windows_setup import update_windows

from setup.robot_params import robot, params, inputs
from setup.robot_params import send_command

from setup.fused_pipelines import stepTimingPipeline
from setup.fused_pipelines import handle

from setup.expectations import Expectation

enable_writing = False

if enable_writing:
    from setup.data_writing import filepath
    from setup.data_writing import update_dataframe
    from setup.data_writing import save

phase = 0.0

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
    
imu = robot.getDevice("imu")
imu.enable(timestep)

# save current frequency before step timing changes
prev_freq = params.freq

while robot.step(timestep) != -1:
    update_windows()

    # return current frequency to its initial value
    params.freq = prev_freq

    deviations = np.array(imu.getRollPitchYaw()[:2]) - Expectation(phase).angles()

    inputs.fusedErrors = handle(deviations)

    timingFreqDelta = stepTimingPipeline.handle(deviations, phase)
    params.freq += timingFreqDelta
        
    outputs = sk.IKWalkOutputs()
    if enable_writing: 
        update_dataframe(imu, Expectation(phase).angles(), phase, inputs, stepTimingPipeline.timing_weight, timingFreqDelta)

        if not filepath.exists():
            save()

    if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs, inputs):
        send_command(outputs)
        phase = outputs.phase
    else:
        print(" Inverse Kinematics error. Position not reachable.", file=stderr)
