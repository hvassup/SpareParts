from shared.route_planner import turn_to_point
from shared.util import sensor_readings_to_motor_speeds

phase = 0  # Should start at 0

def move(robot):
    pass

def look_for_april_tag():
    return 1, -1

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
    return 1 - spiral_idx / 100, 1


def explore(x, y, q, target_pos, sensors):
    sensor_left_mult, sensor_right_mult = sensor_readings_to_motor_speeds(sensors)
    return sensor_left_mult, sensor_right_mult


def follow_path():
    pass


def get_wheel_speeds(x, y, q, target_pos, sensors, b1, b2):
    if phase == 0:
        return find_current_position()
    elif phase == 1:
        return explore(x, y, q, target_pos, sensors)
    elif phase == 2:
        return follow_path()
    else:
        # if b1 and not b2:
        #     return 0, 0.2
        # elif b2 and not b1:
        #     return 0.2, 0 
        # elif b1 and b2:
        #     return -0.2, -0.2
        # else:
        sensor_left_mult, sensor_right_mult = sensor_readings_to_motor_speeds(sensors)
        path_left_mult, path_right_mult = turn_to_point(x, y, q, *target_pos)
        left_mult = sensor_left_mult * path_left_mult
        right_mult = sensor_right_mult * path_right_mult

        return left_mult, right_mult
