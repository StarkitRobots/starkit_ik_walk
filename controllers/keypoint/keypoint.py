"""keypoint controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import json
from controller import Robot

def deg2rad(deg):
    return deg / 180 * math.pi
def rad2deg(rad):
    return rad / math.pi * 180


def read_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data
    
def interpolate_points(start, end):
    interpolated_points = []
    for i in range(steps):
        point = start + (end - start) / steps * i
        interpolated_points.append(point)
    print(interpolated_points)
    return interpolated_points


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motion = read_json("move.json")

left_elbow = robot.getDevice("left_elbow")
left_elbow_sensor = robot.getDevice("left_elbow_sensor")
left_elbow_sensor.enable(timestep)

current_point = 0
current_sub_point = 0
steps = 10
target = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
robot.step(timestep)
interpolated_points = interpolate_points(left_elbow_sensor.getValue(), 
                                         deg2rad(motion["left_elbow"][0][1]))

while robot.step(timestep) != -1:
    if current_point < motion["over"][1][0]:
        # if current_sub_point > steps-1:
        # current_sub_point = 0
        current_point += 1
        interpolated_points = interpolate_points(deg2rad(motion["left_elbow"][current_point - 1][1]), 
                                         deg2rad(motion["left_elbow"][current_point][1]))
    if current_sub_point > steps-1: 
        current_sub_point = steps-1
    else:
        current_sub_point += 1
    print(current_sub_point)
    target = interpolated_points[current_sub_point]
    
    
    left_elbow.setPosition(target)
    #print(current_point, motion["left_elbow"][current_point][1], rad2deg(left_elbow_sensor.getValue()))

# Enter here exit cleanup code.
