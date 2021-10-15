import math

from shared.util import deg_to_rad, rad_to_deg

## Convert lidar distances, to world coordinates
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
