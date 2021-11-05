import math
from random import random
from statistics import mean

import cv2
import numpy as np


# return a number between -span/2 - span/2
def rand(span):
    return random() * span - span / 2


def normalize_lidar(x):
    if x > 0.20:
        return 0
    else:
        return 1 - x * 3

def sensor_readings_to_motor_speeds(sensors):
    # 5025
    sensor1, sensor2, sensor3, sensor4, sensor5 = list(map(lambda x: x / 5025, sensors))

    left_mult = 1 - (sensor4 + sensor5) + sensor3 / 10
    right_mult = 1 - (sensor1 + sensor2) - sensor3 / 10
    return left_mult, right_mult


import math
# Get distance between two points
def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

euclidean_distance(0, 0, 0.005069819708289762, 0.01506650516720775)

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
        angle = math.radians(i)
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

def find_robot_pos2(center, angle):
    rx, ry = rotate_point(*center, angle)
    return -rx, -ry


# Scale up points and calculate the best fitting rectangle
# Scale up because only ints are supported
def calc_rectangle(points):
    scalar = 1000
    _points = list(map(lambda p: (p[0] * scalar, p[1] * scalar), points))
    (cx, cy), (w, h), angle = cv2.minAreaRect(np.asarray(_points).astype(np.int))
    return (cx / scalar, cy / scalar), (w / scalar, h / scalar), angle


def is_point_inside_rectangle(px, py, pos, size):
    rx, ry = pos
    rw, rh = size
    if px >= rx and py >= ry and px <= rx + rw and py <= ry + rh:
        return True


def rolling_average(values, new_value):
    values.insert(0, new_value)
    del values[5]
    return mean(values)


def moving_average(a, n=3):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


def angle_difference(a1, a2):
    diff = a1 - a2
    return (diff + math.pi) % (2 * math.pi) - math.pi
