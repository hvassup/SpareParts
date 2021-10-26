from shapely.geometry.polygon import LinearRing

# Constants

R = 0.043  # radius of wheels in meters
L = 0.092  # distance between wheels in meters

W = 192.0  # width of arena
H = 113.0  # height of arena

# the world is a rectangular arena with width W and height H
world = LinearRing([(W / 2, H / 2), (-W / 2, H / 2), (-W / 2, -H / 2), (W / 2, -H / 2)])

robot_timestep = 0.1  # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch)