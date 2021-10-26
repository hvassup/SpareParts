from math import sqrt, cos, sin, pi

from shapely.geometry.linestring import LineString

def distance_to_sensor_reading(distance):
    """
    Takes a distance in meters and returns a simulated sensor reading
    Parameters
        ----------
        distance : number
            The distance from the robot to the world in meters
    Returns
    -------
        float
            number between 0 and 5020
    """
    distance -= 0.05  # Simulate, that the sensors are not perfectly centered on the robot

    max_dist = 0.2
    max_reading = 5020

    if distance > max_dist:
        return 0
    return (1 - (distance / max_dist)) * max_reading

def read_april_tag(angle):
    """
    Takes an angle and simulates what april tag the robot would be looking at
    April tags go from 0 -> 15
    0 is on the middle on the right
    4 is on the middle of the bottom
    8 is on the middle of the left
    12 is on the middle of the top
    """

def get_sensor_distance(world, W, H, x, y, q, angle):
    """
    Shoot a ray from the robot in a direction, and return distance
    :param angle: the angle relative to the robot's angle
    :return: distance from robot to world
    """
    angle = angle / 180 * pi  # Convert to radians
    # simple single-ray sensor
    scalar = W + H
    ray = LineString([(x, y), (x + cos(q + angle) * scalar, (y + sin(q + angle) * scalar))])
    # a line from robot to a point outside arena in direction of q
    s = world.intersection(ray)

    distance = sqrt((s.x - x) ** 2 + (s.y - y) ** 2)  # distance to wall

    return distance

def get_lidar(world, W, H, x, y, q, resolution):
    """
    Shoot rays out in all directions
    :param resolution: How many rays to shoot out (Default 360)
    :return: a list of distances from the robot, to the world
    """
    return [get_sensor_distance(world, W, H, x, y, q, 360 / resolution * i) for i in range(0, resolution)]

