import math

def get_lidar_points(readings):
    resolution = len(readings)
    points = []
    for _i, reading in enumerate(readings):
        i = 360 / resolution * _i
        angle = i / 180 * math.pi  # Convert to radians
        _x = math.cos(angle) * reading
        _y = math.sin(angle) * reading
        points.append((float(_x), float(_y)))
    return points
