# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import starkit_ik_walk as sk

params = sk.IKWalkParameters()

params.distHipToKnee = 0.141
params.distKneeToAnkle = 0.14
params.distAnkleToGround = 0.042
params.distFeetLateral = 0.092
params.freq = 2.0
params.enabledGain = 0.0
params.supportPhaseRatio = 0.0
params.footYOffset = 0.03
params.stepGain = 0.0
params.riseGain = 0.04
params.turnGain = 0.0
params.lateralGain = 0.0
params.trunkZOffset = 0.02
params.swingGain = 0.02
params.swingRollGain = 0.0
params.swingPhase = 0.25
params.stepUpVel = 4.0
params.stepDownVel = 4.0
params.riseUpVel = 4.0
params.riseDownVel = 4.0
params.swingPause = 0.0
params.swingVel = 4.0
params.trunkXOffset = 0.02
params.trunkYOffset = 0.0
params.trunkPitch = 0.15
params.trunkRoll = 0.0
params.extraLeftX = 0.0
params.extraLeftY = 0.0
params.extraLeftZ = 0.0
params.extraRightX = 0.0
params.extraRightY = 0.0
params.extraRightZ = 0.0
params.extraLeftYaw = 0.0
params.extraLeftPitch = 0.0
params.extraLeftRoll = 0.0
params.extraRightYaw = 0.0
params.extraRightPitch = 0.0
params.extraRightRoll = 0.0

labels = {}

attribute_ranges = {
    "stepGain": (-0.1, 0.1),
    "lateralGain": (-0.06, 0.06),
    "turnGain": (-0.5, 0.5),
    "freq": (0.1, 5.0),
    "supportPhaseRatio": (0.0, 1.0),
    "footYOffset": (-0.2, 0.2),
    "riseGain": (0.0, 0.1),
    "swingGain": (0.0, 0.1),
    "swingRollGain": (-1.0, 1.0),
    "swingPhase": (0.0, 1.0),
    # "stepUpVel": (0.0, 5.0),
    # "stepDownVel": (0.0, 5.0),
    # "riseUpVel": (0.0, 5.0),
    # "riseDownVel": (0.0, 5.0),
    "swingPause": (0.0, 0.5),
    "swingVel": (0.0, 5.0),
    "trunkXOffset": (-0.2, 0.2),
    "trunkYOffset": (-0.2, 0.2),
    "trunkZOffset": (0.01, 0.2),
    "trunkPitch": (-1.0, 1.0),
    "trunkRoll": (-1.0, 1.0)
}

# double support duration 0.1

inputs_ranges = {
    "deadbandRadius": (0.01, 1.0),
    "gainSpeedUp": (0.0, 15.0),
    "gainSlowDown": (0.0, 15.0),
    "weightFactor": (0.0, 5.0),
    "doubleSupportPhaseLen": (-2 * 3.14, 2 * 3.14),
}

dof_names = [
    'left_elbow',
    'right_elbow',
    'left_hip_yaw', 
    'left_hip_roll', 
    'left_hip_pitch', 
    'left_knee', 
    'left_ankle_pitch', 
    'left_ankle_roll', 
    'right_hip_yaw', 
    'right_hip_roll', 
    'right_hip_pitch', 
    'right_knee', 
    'right_ankle_pitch', 
    'right_ankle_roll',
    # 'left_shoulder_roll',
    'left_shoulder_pitch',
    # 'right_shoulder_roll',
    'right_shoulder_pitch',
]

# create the Robot instance.
robot = Robot()

servos = {}
for name in dof_names:
    servos[name] = robot.getDevice(name)

def send_command(command: sk.IKWalkOutputs):
    for name, motor in servos.items():
        # if "elbow" in name:
        #     motor.setPosition(-2.5)
        # else:
        motor.setPosition(getattr(command, name))

inputs = sk.IKWalkInputs()

inputs.fusedErrorParams.fusedFeedbackX = 0.0
inputs.fusedErrorParams.fusedFeedbackY = 0.0
inputs.fusedErrorParams.fusedDerivativeFeedbackX = 0.0
inputs.fusedErrorParams.fusedDerivativeFeedbackY = 0.0
inputs.fusedErrorParams.fusedIntegralFeedbackX = 0.0
inputs.fusedErrorParams.fusedIntegralFeedbackY = 0.0

inputs.fusedErrors.fusedErrorX = 0.0
inputs.fusedErrors.fusedErrorY = 0.0
inputs.fusedErrors.fusedDerivativeErrorX = 0.0
inputs.fusedErrors.fusedDerivativeErrorY = 0.0
inputs.fusedErrors.fusedIntegralErrorX = 0.0
inputs.fusedErrors.fusedIntegralErrorY = 0.0

inputs.armParams.biasArmAngleX = 0.0
inputs.armParams.biasArmAngleY = 0.0
inputs.armParams.fusedFeedbackArmAngleX = 0.0
inputs.armParams.fusedFeedbackArmAngleY = 0.2 #0.2, 0.2
inputs.armParams.fusedDerivativeFeedbackArmAngleX = 0.0
inputs.armParams.fusedDerivativeFeedbackArmAngleY = 2.4 #0.3, 2.4
inputs.armParams.fusedIntegralFeedbackArmAngleX = 0.0
inputs.armParams.fusedIntegralFeedbackArmAngleY = 0.0 #-1.0, 0.0

inputs.armParams.extension = 0.09

inputs.stepTiming.deadbandRadius = 0.02
inputs.stepTiming.gainSpeedUp = 5.0
inputs.stepTiming.gainSlowDown = 10.3
inputs.stepTiming.weightFactor = 1.4
inputs.stepTiming.doubleSupportPhaseLen = 3.14
inputs.stepTiming.gaitFrequencyMax = 2.4