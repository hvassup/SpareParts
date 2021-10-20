import math
from re import A

from numpy import diff

from shared.util import clamp
test_route = []

def turn_to_point(currX, currY, angle, toX, toY):
    angle_margin = 0.1
    
    to_angle = math.pi / 2 - math.atan2(toX - currX, toY - currY)
    angle = angle % (2 * math.pi)
    
    angle_diff = to_angle - angle
    angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
    
    # return smooth_movement(angle_diff, angle_margin)
    return turn_and_go(angle_diff, angle_margin)
    
def turn_and_go(angle_diff, angle_margin):
    if angle_diff < -angle_margin:
        return clamp(abs(angle_diff), 0, 1), -clamp(abs(angle_diff), 0, 1)
    elif angle_diff > angle_margin:
        return -clamp(abs(angle_diff), 0, 1), clamp(abs(angle_diff), 0, 1)
    else:
        return 1, 1

def smooth_movement(angle_diff, angle_margin):
    if angle_diff < -angle_margin:
        return 1, 1 - clamp(abs(angle_diff), 0, 2)
    elif angle_diff > angle_margin:
        return 1 - clamp(abs(angle_diff), 0, 2), 1
    else:
        return 1, 1