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
    
def interpolate(points):
    x = []
    y = []
    for point in points:
        x.append(point[0])
        y.append(point[1])
    return interp1d(x, y, fill_value='extrapolate')
    
    


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motion = read_json("move.json")

left_elbow = robot.getDevice("left_elbow")
left_elbow_sensor = robot.getDevice("left_elbow_sensor")
left_elbow_sensor.enable(timestep)


robot.step(timestep)

timer = 0
trajectory = interpolate(motion["left_elbow"])
print(trajectory(0), trajectory(0.5), trajectory(0.6), trajectory(1))

while robot.step(timestep) != -1:
    timer += timestep / 1000 
    if timer < motion["over"][1][0]:
        target = trajectory(timer)
    left_elbow.setPosition(deg2rad(target))
    #print(current_point, motion["left_elbow"][current_point][1], rad2deg(left_elbow_sensor.getValue()))

# Enter here exit cleanup code.
