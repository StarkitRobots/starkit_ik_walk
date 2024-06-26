from sys import stderr
from pathlib import Path

import numpy as np
import pandas as pd

import starkit_ik_walk as sk

from utils.AnglesConverter  import AnglesConverter

from setup.pulseSetup import pulseGen, enableRandomPulse

from stabilization.PIDStab      import PIDStabilization
from stabilization.PressureStab import PressureStab

import setup.setup

from setup.windows_setup import update_windows

from setup.robot_params import robot, params, inputs
from setup.robot_params import send_command

enableStabilization         = False
enablePressureStabilization = True
enableWriting               = False

if enableWriting:
    from setup.data_writing import filepath
    from setup.data_writing import update_dataframe
    from setup.data_writing import save

robotBody = robot.getFromDef('sahr')

# print(dir(robot))
# print(dir(robotBody))

conv = AnglesConverter()

pulseDirection = np.array([0.0, 1.0, 0.0])
pulsePeriod = 0.5   # sec
pulseLength = 0.2   # sec
pulseAmpl = 2.0     # N

# pulseGen = PeriodicPulseGenerator(pulseDirection, pulsePeriod, pulseLength, pulseAmpl)

PIDStab = PIDStabilization()
PIDStab.params.defaultFusedPitch = -params.trunkPitch

pressStab = PressureStab(params.freq)
phase = 0.0

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# pulseGen = RandomPulseGenerator(pulseDirection, pulseAmpl, pulseLength, 10.0, 5)
    
imu = robot.getDevice("imu")
imu.enable(timestep)

if enablePressureStabilization:
    pressureSensorsRight = []
    pressureSensorsLeft = []

    for i in range(4):
        pressureSensorsRight.append(robot.getDevice(f'/right_pressure/pressure_{i}'))
        pressureSensorsRight[-1].enable(timestep)

        pressureSensorsLeft.append(robot.getDevice(f'/left_pressure/pressure_{i}'))
        pressureSensorsLeft[-1].enable(timestep)

# save current frequency before step timing changes
prev_freq = params.freq

while robot.step(timestep) != -1:

    # pulseGen.ampl += 0.001
    update_windows()
    
    outputs = sk.IKWalkOutputs()

    if enableRandomPulse:
        pulse = pulseGen.tick(timestep / 1000)
        print(pulse)

        robotBody.addForce(pulse.tolist(), False)

    # if pressStab.isLeftSupport and pressStab.timeSinceLastStep / pressStab.stepDuration > 0.5:
    #     robotBody.addForce([0.0, 7.0, 0.0], False)

    if params.enabledGain > 0.0:
        pressStab.startWalking()
    else:
        pressStab.stopWalking()

    # arms position calucalted using pid stab approach and params
    alpha = np.arccos(1 - PIDStab.params.armExtension)

    outputs.left_shoulder_pitch     = alpha
    outputs.right_shoulder_pitch    = alpha

    outputs.left_elbow  = -2.0 * alpha
    outputs.right_elbow = -2.0 * alpha
    
    # return current frequency to its initial value
    params.freq = prev_freq
    
    if enableStabilization:
        fusedYaw, fusedPitch, fusedRoll, fusedHemi = conv.euler2fused(*list(reversed(imu.getRollPitchYaw())))

        fusedRollDeviation  = fusedRoll - PIDStab.expectedFusedRoll(phase)
        fusedPitchDeviation = fusedPitch - PIDStab.expectedFusedPitch(phase)

        freqDelta   = PIDStab.stepTimingFeedback(fusedRollDeviation, phase)
        armFeedback = PIDStab.armFeedback(fusedPitchDeviation, timestep / 1000)

        outputs.left_shoulder_pitch     += armFeedback
        outputs.right_shoulder_pitch    += armFeedback

        params.freq = np.clip(params.freq + freqDelta, 0.0, PIDStab.params.gaitFrequencyMax)
        
    # if enableWriting: 
    #     update_dataframe(imu, Expectation(phase).angles(), phase, inputs, stepTimingPipeline.timing_weight, timingFreqDelta)

    #     if not filepath.exists():
    #         save()
    
    # print(params.freq)

    if enablePressureStabilization:
        phase = pressStab.getCurrentPhase(
            np.array([pressureSensor.getValue() for pressureSensor in pressureSensorsLeft]),
            np.array([pressureSensor.getValue() for pressureSensor in pressureSensorsRight]),
            timestep / 1000
        )

    if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs, inputs):
        send_command(outputs)
        phase = outputs.phase
    else:
        print(" Inverse Kinematics error. Position not reachable.", file=stderr)
