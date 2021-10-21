import random

from cv2 import SparsePyrLKOpticalFlow
from shared.route_planner import turn_to_point
from shared.util import clamp, sensor_readings_to_motor_speeds

phase = 3 # Should start at 0

def combine_speeds(s1, s2):
    l1, r1 = s1
    l2, r2 = s2
    return l1 * l2, r1 * r2

def find_current_position():
    pass

spiral_idx = 0
def spiral():
    global spiral_idx
    spiral_idx += 1
    return 1 - spiral_idx/100, 1

def explore(x, y, q, target_pos, sensors):
    sensor_left_mult, sensor_right_mult = sensor_readings_to_motor_speeds(sensors)
    return sensor_left_mult, sensor_right_mult

def follow_path():
    pass

def get_wheel_speeds(x, y, q, target_pos, sensors):
    if phase == 0:
        return find_current_position()
    elif phase == 1:
        return explore(x, y, q, target_pos, sensors)
    elif phase == 2:
        return follow_path()
    else:
        sensor_left_mult, sensor_right_mult = sensor_readings_to_motor_speeds(sensors)
        path_left_mult, path_right_mult = turn_to_point(x, y, q, *target_pos)
        left_mult = sensor_left_mult * path_left_mult
        right_mult = sensor_right_mult * path_right_mult

        return left_mult, right_mult