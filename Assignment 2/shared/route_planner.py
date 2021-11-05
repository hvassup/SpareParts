import math

from shared.util import clamp, angle_difference

test_route = []

def angle_to_point(currX, currY, angle, toX, toY):
    
    to_angle = math.atan2(toY - currY, toX - currX)
    angle = angle % (2 * math.pi)

    return angle_difference(to_angle, angle)

def turn_to_point(currX, currY, angle, toX, toY, distance):
    angle_margin = 0.3

    angle_diff = angle_to_point(currX, currY, angle, toX, toY)

    # return smooth_movement(angle_diff, angle_margin)
    # return turn_and_go(angle_diff, angle_margin)

    # if distance < 0.15:
    #     return fixed_turn_and_go(angle_diff, angle_margin)

    if abs(angle_diff) > angle_margin:
        return fixed_turn_and_go(angle_diff, angle_margin)
    else:
        return smooth_movement(angle_diff)

def fixed_turn_and_go(angle_diff, angle_margin, turn_multiplier=0.3):
    if angle_diff < -angle_margin:
        return -turn_multiplier, turn_multiplier
    elif angle_diff > angle_margin:
        return turn_multiplier, -turn_multiplier
    else:
        return 1, 1

def turn_and_go(angle_diff, angle_margin, turn_multiplier=0.2):
    if angle_diff < -angle_margin:
        return -clamp(abs(angle_diff) * turn_multiplier, 0, 1), clamp(abs(angle_diff) * turn_multiplier, 0, 1)
    elif angle_diff > angle_margin:
        return clamp(abs(angle_diff) * turn_multiplier, 0, 1), -clamp(abs(angle_diff) * turn_multiplier, 0, 1)
    else:
        return 1, 1


def smooth_movement(angle_diff):
    if angle_diff < 0:
        return 1 - abs(angle_diff) * 1.5, 1
    elif angle_diff > 0:
        return 1, 1 - abs(angle_diff) * 1.5
    else:
        return 1, 1
