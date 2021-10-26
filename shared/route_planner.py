import math

from shared.util import clamp
test_route = []

def angle_to_point(currX, currY, angle, toX, toY):    
    to_angle = math.pi / 2 - math.atan2(toX - currX, toY - currY)
    angle = angle % (2 * math.pi)
    
    angle_diff = to_angle - angle
    return (angle_diff + math.pi) % (2 * math.pi) - math.pi

def turn_to_point(currX, currY, angle, toX, toY):
    angle_margin = 0.1
    
    angle_diff = angle_to_point(currX, currY, angle, toX, toY)

    # return smooth_movement(angle_diff, angle_margin)
    return turn_and_go(angle_diff, angle_margin)
    
def turn_and_go(angle_diff, angle_margin, turn_multiplier=1):
    if angle_diff < -angle_margin:
        return clamp(abs(angle_diff) * turn_multiplier, 0, 1), -clamp(abs(angle_diff) * turn_multiplier, 0, 1)
    elif angle_diff > angle_margin:
        return -clamp(abs(angle_diff) * turn_multiplier, 0, 1), clamp(abs(angle_diff) * turn_multiplier, 0, 1)
    else:
        return 1, 1

def smooth_movement(angle_diff, angle_margin):
    if angle_diff < -angle_margin:
        return 1, 1 - clamp(abs(angle_diff), 0, 2)
    elif angle_diff > angle_margin:
        return 1 - clamp(abs(angle_diff), 0, 2), 1
    else:
        return 1, 1