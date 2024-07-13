from sys import stderr
from pathlib import Path

import numpy as np
import pandas as pd

import starkit_ik_walk as sk

from utils.AnglesConverter  import AnglesConverter

from setup.pulseSetup import pulseGen, enableRandomPulse
from setup.stabSetup import PIDStab, pressStab

from stabilization.PressureStab import PressureStab

from setup.windows_setup import update_windows

from setup.robot_params import robot, params, inputs
from setup.robot_params import send_command

enableStabilization         = True
enablePressureStabilization = False
enableWriting               = False

if enableWriting:
    from setup.data_writing import filepath
    from setup.data_writing import update_dataframe
    from setup.data_writing import save

robotBody = robot.getFromDef('sahr')

conv = AnglesConverter()
pressStab = PressureStab(params.freq)
# PIDStab.params.defaultFusedPitch = -params.trunkPitch

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
prevTrunkPitch = params.trunkPitch
prevStepGain = params.stepGain

while robot.step(timestep) != -1:

    update_windows()
    
    if pulseGen.time + timestep / 1000 >= pulseGen.timeOverall:
        print("Passed", pulseGen.ampl, "N")
        # pulseGen.ampl += 0.5
    
    outputs = sk.IKWalkOutputs()

    if enableRandomPulse:
        pulse = pulseGen.tick(timestep / 1000)
        print(pulse)
        if np.linalg.norm(pulse) > 0.0:
            print("PUSH")
        else:
            print("To next push: ", pulseGen.timeToNextPulse())
        robotBody.addForce(pulse.tolist(), True)

    # if pressStab.isLeftSupport and pressStab.timeSinceLastStep / pressStab.stepDuration > 0.5:
    #     robotBody.addForce([0.0, 7.0, 0.0], False)

    if enablePressureStabilization and params.enabledGain > 0.0:
        pressStab.startWalking()
    elif enablePressureStabilization:
        pressStab.stopWalking()

    # arms position calucalted using pid stab approach and params
    alpha = np.arccos(1 - PIDStab.params.armExtension)

    outputs.left_shoulder_pitch     = alpha
    outputs.right_shoulder_pitch    = alpha

    outputs.left_elbow  = -2.0 * alpha
    outputs.right_elbow = -2.0 * alpha
    
    # return current frequency to its initial value
    params.freq = prev_freq
    params.trunkPitch = prevTrunkPitch
    
    if enableStabilization:
        fusedYaw, fusedPitch, fusedRoll, fusedHemi = conv.euler2fused(*list(reversed(imu.getRollPitchYaw())))

        fusedPitch = np.clip(fusedPitch, -0.7, 0.09)

        fusedRollDeviation  = fusedRoll - PIDStab.expectedFusedRoll(phase)
        fusedPitchDeviation = fusedPitch - PIDStab.expectedFusedPitch(phase)

        PIDStab.updateFeedback(fusedRollDeviation, fusedPitchDeviation, timestep / 1000)
        freqDelta   = PIDStab.stepTimingFeedback(fusedRollDeviation, phase)
        
        hipRollFeedback     = PIDStab.fusedRollHipFeedback()
        comFeedback         = PIDStab.fusedRollCOMShiftFeedback()
        armFeedback         = PIDStab.fusedPitchArmFeedback()
        hipPitchFeedback    = PIDStab.fusedPitchHipFeedback()

        outputs.left_shoulder_pitch     += armFeedback
        outputs.right_shoulder_pitch    += armFeedback

        params.freq = np.clip(params.freq + freqDelta, 0.0, PIDStab.params.gaitFrequencyMax)

        # params.trunkPitch += hipPitchFeedback
        # params.stepGain -= hipPitchFeedback / 5

        print("hipPitchFeedback", hipPitchFeedback)
        print("Trunk Pitch", params.trunkPitch)
        # print("Step Gain", params.stepGain)
        print("fusedPitch", fusedPitch)
        print("fusedPitchDeviation", fusedPitchDeviation)

        # print("COM Feedback", comFeedback)
        if params.enabledGain > 0.0: # state == isWalking
            params.trunkYOffset = comFeedback * np.sign(-fusedRoll * fusedRollDeviation)
            params.trunkRoll = hipRollFeedback * np.sign(fusedRoll * fusedRollDeviation)
            
            # print("Fused Roll", fusedRoll)
            # print("Fused Roll Deviation", fusedRollDeviation)
            # print("TrunkRoll", params.trunkRoll)
        
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
        outputs.left_hip_pitch -= hipPitchFeedback
        outputs.right_hip_pitch -= hipPitchFeedback

        outputs.left_ankle_pitch += hipPitchFeedback
        outputs.right_ankle_pitch += hipPitchFeedback
        send_command(outputs)

        # params.stepGain += hipPitchFeedback / 5
        phase = outputs.phase
    else:
        print(" Inverse Kinematics error. Position not reachable.", file=stderr)
