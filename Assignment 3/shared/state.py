from math import sqrt
from shapely.geometry.polygon import LinearRing
from random import random
# Constants

R = 0.043  # radius of wheels in meters
L = 0.092  # distance between wheels in meters

W = 1.5  # width of arena
H = 1.5  # height of arena

# the world is a rectangular arena with width W and height H
# world = LinearRing([(W / 2, H / 2), (-W / 2, H / 2), (-W / 2, -H / 2), (W / 2, -H / 2)])
world = LinearRing([(-0.5, -0.5), (-0.5, 0.5), (0, 0.2), (0.5, 0.5), (0.5, -0.5), (0, -0.2), (-0.5, -0.5)])
# obstacle_size = 0.2
obstacles = []
# n = 4
# for i in range(0, n):
#     sqr = sqrt(n)
#     # x = random() * W - W / 2
#     # y = random() * H - H / 2
#     x = ((i % sqr) / sqr) * W - W / 2 #  + (W / (sqr * 2)) - obstacle_size / 2
#     y = ((i // sqr) / sqr) * H - H / 2 #  + (H / (sqr * 2)) - obstacle_size / 2
#     obstacles.append(LinearRing([(x, y), (x + obstacle_size, y), (x + obstacle_size, y + obstacle_size), (x, y + obstacle_size)]))

# environment = world

# for obstacle in obstacles:
#     environment = environment.intersection(obstacle)

april_tags = [(W / 2, 0), (W / 2, H / 3),
              (W * 2 / 5, H / 2), (W / 5, H / 2), (0, H / 2), (-W / 5, H / 2), (-W * 2 / 5, H / 2),
              (-W / 2, H / 3), (-W / 2, 0), (-W / 2, -H / 3),
              (-W * 2 / 5, -H / 2), (-W / 5, -H / 2), (0, -H / 2), (W / 5, -H / 2), (W * 2 / 5, -H / 2),
              (W / 2, -H / 3)]

april_tag_angles = [
    0, 0,
    90, 90, 90, 90, 90,
    180, 180, 180,
    270, 270, 270, 270, 270,
    0
]

robot_timestep = 0.1  # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch)
