from pathlib import Path

import numpy as np
import pandas as pd

from setup.robot_params import robot

size = int(robot.getBasicTimeStep() * 100)

filepath = Path("angles.csv")

df = pd.DataFrame(
    {
        "timestep": np.zeros(size),

        # angles
        "roll":     np.zeros(size),
        "pitch":    np.zeros(size),
        
        # expected angles
        "expected_roll":    np.zeros(size),
        "expected_pitch":   np.zeros(size),

        #phase
        "phase": np.zeros(size),

        # arm params from feedback
        "fused_arm_Y":  np.zeros(size),
        "dFused_arm_Y": np.zeros(size),
        "iFused_arm_Y": np.zeros(size),

        # step timing
        "timing_feed":  np.zeros(size),
        "freq_delta":   np.zeros(size),
    }
)

def update_timestep():
    global df
    df["timestep"] = np.append(df["timestep"][1:], df["timestep"].iloc[-1] + 1)

def update_angles(imu):
    global df

    df["roll"]  = np.append(df["roll"]  [1:], imu.getRollPitchYaw()[0])
    df["pitch"] = np.append(df["pitch"] [1:], imu.getRollPitchYaw()[1])

def update_expected_angles(expected_angles):
    global df

    df["expected_roll"]     = np.append(df["expected_roll"] [1:], expected_angles[0])
    df["expected_pitch"]    = np.append(df["expected_pitch"][1:], expected_angles[1])

def update_phase(current_phase):
    global df

    df["phase"] = np.append(df["phase"][1:], current_phase)

def update_arm_params(inputs):
    global df

    df["fused_arm_Y"]   = np.append(df["fused_arm_Y"]   [1:], inputs.fusedErrors.fusedErrorY            * inputs.armParams.fusedFeedbackArmAngleY           )
    df["dFused_arm_Y"]  = np.append(df["dFused_arm_Y"]  [1:], inputs.fusedErrors.fusedDerivativeErrorY  * inputs.armParams.fusedDerivativeFeedbackArmAngleY )
    df["iFused_arm_Y"]  = np.append(df["iFused_arm_Y"]  [1:], inputs.fusedErrors.fusedIntegralErrorY    * inputs.armParams.fusedIntegralFeedbackArmAngleY   )

def update_step_timing_params(timing_feed, freq_delta):
    global df

    df["timing_feed"] = np.append(df["timing_feed"] [1:], timing_feed   )
    df["freq_delta"] = np.append(df["freq_delta"]   [1:], freq_delta    )

def update_dataframe(imu, expected_angles, current_phase, inputs, timing_feed, freq_delta):
    update_timestep()
    update_angles(imu)
    update_expected_angles(expected_angles)
    update_phase(current_phase)
    update_arm_params(inputs)
    update_step_timing_params(timing_feed, freq_delta)

def save():
    global df
    df.to_csv(filepath, sep=' ')