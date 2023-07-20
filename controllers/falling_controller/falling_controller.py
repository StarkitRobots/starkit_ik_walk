"""falling_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import json
from scipy.interpolate import interp1d
from controller import Robot

def deg2rad(deg):
    return deg / 180 * math.pi
def rad2deg(rad):
    return rad / math.pi * 180

def read_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    refactored_data = {}
    for joint_name in data.keys():
        if joint_name not in ("over", "remap", "head_yaw", "head_pitch"):
            if "left" not in joint_name:
                refactored_data["left_" + joint_name] = data[joint_name]
            if "right" not in joint_name:
                refactored_data["right_" + joint_name] = data[joint_name]
        else:
            refactored_data[joint_name] = data[joint_name]
    return refactored_data
    
def interpolate(motion):
    motion_trajectory = {}
    for joint in motion.items():
        joint_name = joint[0] 
        x = []
        y = []
        points = joint[1]
        for point in points:
            x.append(point[0])
            y.append(point[1])
        if joint_name not in ("over", "remap"):
            joint_trajectory = interp1d(x, y, kind='linear', fill_value='extrapolate')
        elif joint_name == "remap":
            joint_trajectory = interp1d(x, y, kind='next', fill_value='extrapolate')
        else:
            joint_trajectory = interp1d(x, y, kind='zero', fill_value='extrapolate')
        motion_trajectory[joint_name] = joint_trajectory       
    return motion_trajectory
    
def send_commands(commands):
    for joint_name, value in commands.items():
        if joint_name not in ("over", "remap"):
            servos[joint_name].setPosition(deg2rad(value))


def run_standup_front():
    global standup_front_timer
    time_factor = standup_front_trajectory["remap"](standup_front_timer)
    standup_front_timer += time_factor * timestep / 1000 
    if not standup_front_trajectory["over"](standup_front_timer):
        commands = {}
        for joint_name, trajectory in standup_front_trajectory.items(): 
            target = trajectory(standup_front_timer)
            commands[joint_name] = target
        send_commands(commands)
    
def run_standup_back():
    global standup_back_timer
    time_factor = standup_back_trajectory["remap"](standup_back_timer)
    standup_back_timer += time_factor * timestep / 1000 
    if not standup_back_trajectory["over"](standup_back_timer):
        commands = {}
        for joint_name, trajectory in standup_back_trajectory.items(): 
            target = trajectory(standup_back_timer)
            commands[joint_name] = target
        send_commands(commands)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

imu = robot.getDevice("imu")
imu.enable(timestep)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

standup_front_motion = read_json("standup_front.json")

standup_front_timer = 0
standup_front_trajectory = interpolate(standup_front_motion)

standup_back_motion = read_json("standup_back.json")

standup_back_timer = 0
standup_back_trajectory = interpolate(standup_back_motion)

servos = {}

for joint_name in standup_front_trajectory.keys():
    if joint_name not in ("over", "remap"):
        servos[joint_name] = robot.getDevice(joint_name)

state = "ready"
standup_is_over = True
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    roll, pitch, yaw = imu.getRollPitchYaw()
    
    if pitch < -1.0 and standup_is_over:
        state = "fallen_front"
        standup_is_over = False
        standup_back_timer = 0
    elif pitch > 1.0 and standup_is_over:
        state = "fallen_back"
        standup_is_over = False
        standup_front_timer = 0
    elif abs(roll) > 1.0 and standup_is_over:
        state = "fallen_back"
        standup_is_over = False
        standup_front_timer = 0
    elif abs(pitch) < 0.3 and standup_is_over:
        state = "ready"
        standup_front_timer = 0
        standup_back_timer = 0
    
    if state == "fallen_front":
        run_standup_front()
        if standup_front_trajectory["over"](standup_front_timer):
            standup_is_over = True
    elif state == "fallen_back":
        run_standup_back()
        if standup_back_trajectory["over"](standup_back_timer):
            standup_is_over = True
    else:
        pass
        

# Enter here exit cleanup code.
