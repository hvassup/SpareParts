# A prototype simulation of a differential-drive robot with one sensor
import math
from random import random
from sys import path
import cv2
import numpy as np
from numpy import sin, cos, sqrt
from shapely.geometry import LinearRing, LineString, Point
from shared.route_planner import turn_to_point

from simulation.sensor_sim import distance_to_sensor_reading
from simulation.visualization.pygame_visualizer import PyGameVisualizer

from shared.util import calc_rectangle, euclidean_distance, get_lidar_points, round_point, sensor_readings_to_motor_speeds

def get_sensor_distance(angle):
    """
    Shoot a ray from the robot in a direction, and return distance
    :param angle: the angle relative to the robot's angle
    :return: distance from robot to world
    """
    angle = angle / 180 * math.pi  # Convert to radians
    # simple single-ray sensor
    scalar = W + H
    ray = LineString([(x, y), (x + cos(q + angle) * scalar, (y + sin(q + angle) * scalar))])
    # a line from robot to a point outside arena in direction of q
    s = world.intersection(ray)

    distance = sqrt((s.x - x) ** 2 + (s.y - y) ** 2)  # distance to wall

    return distance

def get_lidar(resolution=360):
    """
    Shoot rays out in all directions
    :param resolution: How many rays to shoot out (Default 360)
    :return: a list of distances from the robot, to the world
    """
    return [get_sensor_distance(360 / resolution * i) for i in range(0, resolution)]

def generate_random_danger_spots():
    size = (0.1, 0.1)
    spots = []
    for i in range(0, 10):
        pos = (1 - random() * W, 1 - random() * H)
        spots.append((pos, size))
    return spots

# Constants
R = 0.043  # radius of wheels in meters
L = 0.092  # distance between wheels in meters

W = 2.0  # width of arena
H = 2.0  # height of arena

robot_timestep = 0.1  # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W / 2, H / 2), (-W / 2, H / 2), (-W / 2, -H / 2), (W / 2, -H / 2)])

# Variables

x = 0.0  # robot position in meters - x direction - positive to the right
y = 0.0  # robot position in meters - y direction - positive up
q = math.pi / 2  # robot heading with respect to x-axis in radians

left_wheel_velocity = 5  # robot left wheel velocity in radians/s
right_wheel_velocity = 5  # robot right wheel velocity in radians/s


# Kinematic model

# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot
def simulationstep(_left_wheel_velocity, _right_wheel_velocity):
    global x, y, q

    for step in range(int(robot_timestep / simulation_timestep)):  # step model time/timestep times
        v_x = cos(q) * (R * _left_wheel_velocity / 2 + R * _right_wheel_velocity / 2)
        v_y = sin(q) * (R * _left_wheel_velocity / 2 + R * _right_wheel_velocity / 2)
        omega = (R * _right_wheel_velocity - R * _left_wheel_velocity) / (2 * L)

        x += v_x * simulation_timestep
        y += v_y * simulation_timestep
        q += omega * simulation_timestep

spots = generate_random_danger_spots()

visualizer = PyGameVisualizer()
# visualizer = MatPlotVisualizer()

file = open("trajectory.dat", "w")
turn_counter = 0

def generate_random_path():
    path = []
    for i in range(0, 100):
        _x = (random() * W - W / 2) * 0.8
        _y = (random() * H - H / 2) * 0.8
        path.append((_x, _y))
    return path

path_to_explore = generate_random_path()
path_index = 0

# Simulation loop
for cnt in range(1, 5000):
    # simple controller - change direction of wheels every 10 seconds (100*robot_timestep) unless close to wall then
    # turn on spot
    sensor1 = distance_to_sensor_reading(get_sensor_distance(30))
    sensor2 = distance_to_sensor_reading(get_sensor_distance(15))
    sensor3 = distance_to_sensor_reading(get_sensor_distance(0))
    sensor4 = distance_to_sensor_reading(get_sensor_distance(-15))
    sensor5 = distance_to_sensor_reading(get_sensor_distance(-30))

    lidar_reading = get_lidar(360)
    lidar_points = get_lidar_points(lidar_reading, q)
    center, width_height, _angle = calc_rectangle(lidar_points)
    cx, cy = center

    visualizer.clear()
    visualizer.draw_points(lidar_points, (255, 255, 255), x, y)
    visualizer.draw_points(path_to_explore, (255, 255, 255), 0, 0)
    
    # visualizer.draw_point(-cx, -cy, (0, 0, 255))
    visualizer.draw_point(x, y, (0, 255, 0))

    target_pos = path_to_explore[path_index]
    
    distance_to_goal = euclidean_distance(*target_pos, x, y)
    
    if distance_to_goal < 0.05:
        path_index += 1

    visualizer.draw_point(*target_pos, (0, 255, 255))
    visualizer.draw_line_to_point(*target_pos, x, y)
    
    visualizer.draw_text(f'Actual robot angle:        {math.degrees(q)}', (300, 20))
    visualizer.draw_text(f'Actual robot position:    {round_point(x, y)}', (300, 30))
    visualizer.draw_text(f'Estimated robot position: {round_point(-cx, -cy)}', (300, 40))
    visualizer.draw_text(f'Distance to goal:        {distance_to_goal}', (300, 50))

    visualizer.show()

    sensor_left_mult, sensor_right_mult = sensor_readings_to_motor_speeds(sensor1, sensor2, sensor3, sensor4, sensor5)
    path_left_mult, path_right_mult = turn_to_point(x, y, q, *target_pos)
    left_mult = sensor_left_mult * path_left_mult
    right_mult = sensor_right_mult * path_right_mult
    
    # print(round(path_left_mult, 3), round(path_right_mult, 3))
    
    if cnt % 1000 == 0:
        left_wheel_velocity, right_wheel_velocity = (random(), random())

    if left_wheel_velocity != 1 or right_wheel_velocity != 1:
        turn_counter += 1
        if turn_counter % 100 == 0:
            left_wheel_velocity, right_wheel_velocity = (10, 10)

    # step simulation
    simulationstep(left_wheel_velocity * left_mult, right_wheel_velocity * right_mult)

    # check collision with arena walls
    if world.distance(Point(x, y)) < L / 2:
        file.write(str(x) + ", " + str(y) + ", " + str(cos(q) * 0.05) + ", " + str(sin(q) * 0.05) + "\n")
        print('Collision!', q, x, y)
        print(cnt, 'Steps')
        break

    if cnt % 50 == 0:
        file.write(str(x) + ", " + str(y) + ", " + str(cos(q) * 0.05) + ", " + str(sin(q) * 0.05) + "\n")

file.close()
