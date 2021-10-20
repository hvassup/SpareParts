import math

import cv2
import numpy as np


def sensor_readings_to_motor_speeds(sensor1, sensor2, sensor3, sensor4, sensor5):
    sensor1 /= 5020
    sensor2 /= 5020
    sensor3 /= 5020
    sensor4 /= 5020
    sensor5 /= 5020

    left_mult = 1 - (sensor4 + sensor5) + sensor3 / 10
    right_mult = 1 - (sensor1 + sensor2) - sensor3 / 10
    return left_mult, right_mult

# Convert to radians
def deg_to_rad(deg):
    return deg / 180 * math.pi

# Convert to degrees
def rad_to_deg(rad):
    return rad * 180 / math.pi

# Get distance between two points
def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

# Clamp a number between min_val and max_val
def clamp(n, min_val, max_val):
    return max(min_val, min(n, max_val))

# Round a point
def round_point(x, y, precision=2):
    return round(x, precision), round(y, precision)

# Rotate point around (0, 0)
def rotate_point(x, y, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    x_new = x * c - y * s
    y_new = x * s + y * c
    return x_new, y_new

# Convert lidar distances to points
def get_lidar_points(readings, robot_dir):
    resolution = len(readings)
    points = []
    for _i, reading in enumerate(readings):
        i = 360 / resolution * _i
        angle = deg_to_rad(i)
        x = math.cos(robot_dir + angle) * reading
        y = math.sin(robot_dir + angle) * reading
        points.append((float(x), float(y)))
    return points

# Find robot position given lidar readings
def find_robot_pos(lidar_distances, angle):
    points = get_lidar_points(lidar_distances, 0)
    center, width_height, _angle = cv2.minAreaRect(np.asarray(points).astype(np.int))
    rx, ry = rotate_point(*center, angle)
    return -rx, -ry

# Scale up points and calculate the best fitting rectangle
# Scale up because only ints are supported
def calc_rectangle(points):
    scalar = 1000
    _points = list(map(lambda p: (p[0] * scalar, p[1] * scalar), points))
    (cx, cy), (w, h), angle = cv2.minAreaRect(np.asarray(_points).astype(np.int))
    return (cx / scalar, cy / scalar), (w / scalar, h/scalar), angle