# A prototype simulation of a differential-drive robot with one sensor
import math
from random import random
import cv2
import numpy as np
from numpy import sin, cos, sqrt
from shapely.geometry import LinearRing, LineString, Point
from simulation import visualization
from simulation.sensor_sim import distance_to_sensor_reading
from simulation.visualization.pygame_visualizer import PyGameVisualizer

from shared.util import calc_rectangle, find_robot_pos, get_lidar_points, rad_to_deg, rotate_point, round_point, sensor_readings_to_motor_speeds
import shared.map

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

W = 20.0  # width of arena
H = 20.0  # height of arena

robot_timestep = 0.1  # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W / 2, H / 2), (-W / 2, H / 2), (-W / 2, -H / 2), (W / 2, -H / 2)])

# Variables

x = 8.0  # robot position in meters - x direction - positive to the right
y = 0.0  # robot position in meters - y direction - positive up
q = 0.0  # robot heading with respect to x-axis in radians

left_wheel_velocity = 10  # robot left wheel velocity in radians/s
right_wheel_velocity = 10  # robot right wheel velocity in radians/s


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

# Simulation loop
for cnt in range(5000):
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
    # visualizer.visualize_world(shared.map.map, H, W)
    
    visualizer.draw_points(lidar_points, (255, 255, 255), x, y)
    # visualizer.draw_point(x + cx, y + cy, (255, 0, 0))
    
    # visualizer.draw_line_to_point(cx, cy, x, y)

    # visualizer.visualize_lidar(x, y, q, lidar_reading)
    # visualizer.visualize_safe_zone((-1,q -1), (0.1, 0.1))
    # visualizer.visualize_danger_spots(spots)
    visualizer.draw_point(-cx, -cy, (0, 0, 255))
    visualizer.draw_point(x, y, (0, 255, 0))
    
    visualizer.draw_text(f'Actual robot angle:        {rad_to_deg(q)}', (300, 20))
    visualizer.draw_text(f'Actual robot position:    {round_point(x, y)}', (300, 30))
    visualizer.draw_text(f'Estimated robot position: {round_point(-cx, -cy)}', (300, 40))
    
    # robot_pos = find_robot_pos(lidar_reading, q)

    # visualizer.draw_text(f'estimated robot pos:   {round_point(*robot_pos)}', (300, 40))
    # visualizer.draw_point(*robot_pos, (120, 120, 0))

    visualizer.show()

    left_mult, right_mult = sensor_readings_to_motor_speeds(sensor1, sensor2, sensor3, sensor4, sensor5)
    # print(left_mult, right_mult, sensor3, q * 180 / math.pi)
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
