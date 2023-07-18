"""keypoint controller."""

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
    return data
    
def interpolate(motion):
    motion_trajectory = {}
    for joint in motion.items():
        joint_name = joint[0]
        if joint_name not in ("over", "remap"):
            x = []
            y = []
            points = joint[1]
            for point in points:
                x.append(point[0])
                y.append(point[1])
            joint_trajectory = interp1d(x, y, fill_value='extrapolate')
            motion_trajectory[joint_name] = joint_trajectory
    return motion_trajectory
    
def send_commands(commands):
    for joint_name, value in commands.items():
        servos[joint_name].setPosition(deg2rad(value))


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motion = read_json("move.json")

timer = 0
motion_trajectory = interpolate(motion)

servos = {}

for joint_name in motion_trajectory.keys():
    servos[joint_name] = robot.getDevice(joint_name)

print(motion_trajectory)
#print(trajectory(0), trajectory(0.5), trajectory(0.6), trajectory(1))

while robot.step(timestep) != -1:
    timer += timestep / 1000 
    if timer < motion["over"][1][0]:
        commands = {}
        for joint_name, trajectory in motion_trajectory.items(): 
            target = trajectory(timer)
            commands[joint_name] = target
    send_commands(commands)
    #print(current_point, motion["left_elbow"][current_point][1], rad2deg(left_elbow_sensor.getValue()))

# Enter here exit cleanup code.
