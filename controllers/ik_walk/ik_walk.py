from sys import stderr
import tkinter as tk
from tkinter import ttk

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import os
base_dir = os.getcwd()
from sys import platform
if platform != "linux" and platform != "linux2":
    dll_path = os.path.join(base_dir, "x64")
    os.add_dll_directory(dll_path)
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

phase = 0.0

labels = {}
button_enabled = None

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

def update_value(param_name, value):
    global params
    global labels
    print(float(value))
    setattr(params, param_name, float(value))
    if param_name in labels.keys():
        labels[param_name].configure(text=f"{param_name}: {float(value):.3f}")

def create_window_1():
    global params
    global labels
    global button_enabled
    global attribute_ranges
    window = tk.Tk()
    window.title("Control robot")
    
    button_enabled = ttk.Button(window, text="Enable", command=lambda: toggle_enabled())
    button_enabled.pack(padx=10, pady=10)
    
    for attr_name, (min_value, max_value) in attribute_ranges.items():
        if attr_name in ("stepGain", "lateralGain", "turnGain"):
            label = ttk.Label(window, text=f"{attr_name}: {getattr(params, attr_name):.3f}")
            labels[attr_name] = label
            label.pack(pady=5)
        
            trackbar = ttk.Scale(window, from_=min_value, to=max_value, length=200, orient=tk.HORIZONTAL,
                                 command=lambda value, param_name=attr_name: update_value(param_name, value))
            trackbar.set(getattr(params, attr_name))
            trackbar.pack(pady=5)
    return window

def create_window_2():
    global params
    global labels
    global attribute_ranges
    window = tk.Tk()
    window.title("Parameter settings")
    
    attribute_names = vars(params).keys()
    
    trackbars_per_row = 4
    current_row = 0
    current_column = 0

    for attr_name, (min_value, max_value) in attribute_ranges.items():
        if attr_name not in ("stepGain", "lateralGain", "turnGain"):
            label = ttk.Label(window, text=f"{attr_name}: {getattr(params, attr_name):.3f}")
            label.grid(row=current_row, column=current_column, padx=5, pady=5)
            labels[attr_name] = label

            trackbar = ttk.Scale(window, from_=min_value, to=max_value, length=200, orient=tk.HORIZONTAL,
                                 command=lambda value, param_name=attr_name: update_value(param_name, value))
            trackbar.set(getattr(params, attr_name))
            trackbar.grid(row=current_row, column=current_column+1, padx=5, pady=5)

            current_column += 2
            if current_column >= trackbars_per_row * 2:
                current_row += 1
                current_column = 0
    return window

def toggle_enabled():
    global params
    global labels
    params.enabledGain = 1.0 if params.enabledGain == 0.0 else 0.0
    button_enabled.configure(text="Disable" if params.enabledGain == 1.0 else "Enable")

w1 = create_window_1()
w1.geometry("+1200+200")
w2 = create_window_2()
w2.geometry("+100+550")

# create the Robot instance.
robot = Robot()

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
    'right_ankle_roll']


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
servos = {}
for name in dof_names:
    servos[name] = robot.getDevice(name)

def send_command(command: sk.IKWalkOutputs):
    for name, motor in servos.items():
        if "elbow" in name:
            motor.setPosition(-2.5)
        else:
            motor.setPosition(getattr(command, name))

while robot.step(timestep) != -1:
    w1.update_idletasks()
    w1.update()
    w2.update_idletasks()
    w2.update()
    outputs = sk.IKWalkOutputs()
    if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
        send_command(outputs)
        phase = outputs.phase
    else:
        print(" Inverse Kinematics error. Position not reachable.", file=stderr)