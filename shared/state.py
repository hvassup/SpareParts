import math
from shapely.geometry.polygon import LinearRing

# Constants

R = 0.043  # radius of wheels in meters
L = 0.092  # distance between wheels in meters

W = 1.92  # width of arena
H = 1.13  # height of arena

# the world is a rectangular arena with width W and height H
world = LinearRing([(W / 2, H / 2), (-W / 2, H / 2), (-W / 2, -H / 2), (W / 2, -H / 2)])

april_tags = [(W / 2, 0), (W / 2, H / 3),
(W * 2 / 5, H / 2), (W / 5, H / 2), (0, H / 2), (-W / 5, H / 2), (-W * 2 / 5, H / 2),
(-W / 2, H / 3), (-W / 2, 0), (-W / 2, -H / 3),
(-W * 2 / 5, -H / 2), (-W / 5, -H / 2), (0, -H / 2), (W / 5, -H / 2), (W * 2 / 5, -H / 2), 
(W / 2, -H / 3)]

april_tag_angles = [
    (0), (0), 
    (270), (270), (270), (270), (270), 
    (180), (180), (180),
    (90), (90), (90), (90), (90),
    (0)
]

robot_timestep = 0.1  # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch)