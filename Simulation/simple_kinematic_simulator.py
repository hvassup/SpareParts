# A prototype simulation of a differential-drive robot with one sensor
import math
from random import random

from numpy import sin, cos, sqrt
from shapely.geometry import LinearRing, LineString, Point
from shared.map import Map
from shared.movement import get_wheel_speeds
from shared.route_planner import angle_to_point, turn_to_point
from simulation.bottom_sensor import is_sensor_in_danger_spot

from simulation.sensor_sim import distance_to_sensor_reading
from simulation.visualization.pygame_visualizer import PyGameVisualizer, scale_point

from shared.util import calc_rectangle, clamp, euclidean_distance, get_lidar_points, is_point_inside_rectangle, rotate_point, round_point, sensor_readings_to_motor_speeds

def get_sensor_distance(angle):
    """
    Shoot a ray from the robot in a direction, and return distance
    :param angle: the angle relative to the robot's angle
    :return: distance from robot to world
    """
    angle = angle / 180 * math.pi  # Convert to radians
    # simple single-ray sensor
    scalar = W + H
    ray = LineString([(x, y), (x + cos(q + angle) * scalar, (y + sin(q + angle) * scalar))])
    # a line from robot to a point outside arena in direction of q
    s = world.intersection(ray)

    distance = sqrt((s.x - x) ** 2 + (s.y - y) ** 2)  # distance to wall

    return distance

def get_lidar(resolution=360):
    """
    Shoot rays out in all directions
    :param resolution: How many rays to shoot out (Default 360)
    :return: a list of distances from the robot, to the world
    """
    return [get_sensor_distance(360 / resolution * i) for i in range(0, resolution)]

def generate_random_danger_spots():
    size = (5, 10)
    spots = [((-W / 2 + 2, -H / 2 + 2), size)]
    for i in range(0, 10):
        pos = (W / 2 - random() * (W - size[0]*2), H / 2 - random() * (H - size[1]*2))
        spots.append((pos, size))
    return spots

# Constants
R = 0.043  # radius of wheels in meters
L = 0.092  # distance between wheels in meters

W = 192.0  # width of arena
H = 113.0  # height of arena

robot_timestep = 0.1  # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W / 2, H / 2), (-W / 2, H / 2), (-W / 2, -H / 2), (W / 2, -H / 2)])

# Variables

x = -W/2 + 1.0  # robot position in meters - x direction - positive to the right
y = -H/2 + 1.0  # robot position in meters - y direction - positive up
q = math.pi / 2 + 0.1  # robot heading with respect to x-axis in radians

left_wheel_velocity = 40   # robot left wheel velocity in radians/s
right_wheel_velocity = 40  # robot right wheel velocity in radians/s


# Kinematic model

# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot
def simulationstep(_left_wheel_velocity, _right_wheel_velocity):
    global x, y, q

    for step in range(int(robot_timestep / simulation_timestep)):  # step model time/timestep times
        v_x = cos(q) * (R * _left_wheel_velocity / 2 + R * _right_wheel_velocity / 2)
        v_y = sin(q) * (R * _left_wheel_velocity / 2 + R * _right_wheel_velocity / 2)
        omega = (R * _right_wheel_velocity - R * _left_wheel_velocity) / (2 * L)

        x += v_x * simulation_timestep
        y += v_y * simulation_timestep
        q += omega * simulation_timestep

spots = generate_random_danger_spots()

visualizer = PyGameVisualizer()
# visualizer = MatPlotVisualizer()

file = open("trajectory.dat", "w")
turn_counter = 0

def generate_random_path():
    path = [(x, y)]
    for i in range(0, 100):
        prev_x, prev_y = path[i]
        
        # Random, connected path
        spread = 5
        mul = 0.45
        _x = clamp(prev_x + random() * spread, -W * mul, W * mul)
        _y = clamp(prev_y + random() * spread, -H * mul, H * mul)

        path.append((_x, _y))
    return path

buddy_pos = (W / 2 - 5, 0)

# path_to_explore = generate_random_path()
path_to_explore = [buddy_pos]
path_index = 0

world_map = Map(W, H, 1)

april_tags = [(W / 2, 0), (W / 2, H / 3), 
(W * 2 / 5, H / 2), (W / 5, H / 2), (0, H / 2), (-W / 5, H / 2), (-W * 2 / 5, H / 2),
(-W / 2, H / 3), (-W / 2, 0), (-W / 2, -H / 3),
(-W * 2 / 5, -H / 2), (-W / 5, -H / 2), (0, -H / 2), (W / 5, -H / 2), (W * 2 / 5, -H / 2), 
(W / 2, -H / 3)]

camera_range = 50
camera_fov = 25

bottom_sensor_offset = 3
bottom_sensor_angle = 25

sensor_angles = [30, 15, 0, -15, 30]


# Simulation loop
for cnt in range(1, 5000):
    # simple controller - change direction of wheels every 10 seconds (100*robot_timestep) unless close to wall then
    # turn on spot
    sensors = list(map(lambda x: distance_to_sensor_reading(get_sensor_distance(x)), sensor_angles))
    
    # Init visualization
    visualizer.clear()

    # Draw map
    for safe_spot in world_map.safe_spots:
        res = world_map.RESOLUTION
        visualizer.draw_rectangle(safe_spot, (res, res), (87, 184, 255))
    # for _y in range(0, map.map.shape[1]):
    #     for _x in range(0, map.map.shape[0]):
    #         pos = (_x * res - map.WIDTH / 2, _y * res - map.HEIGHT / 2)
    #         if map.is_danger(*pos):
    #             pass # visualizer.draw_rectangle(pos, (res, res), (254, 28, 28))
    #         else:
    #             visualizer.draw_rectangle(pos, (res, res), (87, 184, 255))

    # Draw lidar
    lidar_reading = get_lidar(50)
    lidar_points = get_lidar_points(lidar_reading, q)
    center, width_height, _angle = calc_rectangle(lidar_points)
    cx, cy = center
    visualizer.draw_points(lidar_points, (255, 255, 255), x, y)

    # Draw camera field of view
    cp1x, cp1y = rotate_point(0, camera_range, q - math.radians(90 + camera_fov))
    cp2x, cp2y = rotate_point(0, camera_range, q - math.radians(90 - camera_fov))
    visualizer.draw_line(x, y, x + cp1x, y + cp1y, (50, 0, 255))
    visualizer.draw_line(x, y, x + cp2x, y + cp2y, (50, 0, 255))    

    # Draw april tags
    for i, pos in enumerate(april_tags):
        visualizer.draw_point(*pos, (255, 255, 0), 3)
        visualizer.draw_text(str(i), scale_point(*pos))
        if euclidean_distance(*pos, x, y) < camera_range and abs(math.degrees(angle_to_point(x, y, q, *pos))) < camera_fov:
            visualizer.draw_line(x, y, *pos, (255, 0 ,0))
            # print(f'I can see {i}')

    # Draw buddy
    visualizer.draw_point(*buddy_pos, (255, 192, 203), 5)
    if euclidean_distance(*buddy_pos, x, y) < camera_range and abs(math.degrees(angle_to_point(x, y, q, *buddy_pos))) < camera_fov:
        break

    # Draw danger spots
    visualizer.draw_rectangles(spots, (120, 0, 0))
    
    # visualizer.draw_point(-cx, -cy, (0, 0, 255))
    # Draw robot
    visualizer.draw_point(x, y, (0, 255, 0), 11)

    # Path finding
    target_pos = path_to_explore[path_index]
    
    distance_to_goal = euclidean_distance(*target_pos, x, y)
    
    if distance_to_goal < 1:
        path_index += 1

    visualizer.draw_points(path_to_explore, (255, 255, 255), 0, 0)
    visualizer.draw_point(*target_pos, (0, 255, 255))
    visualizer.draw_line(*target_pos, x, y, (255, 0, 255))

    # Simulate bottom sensors (They can only see danger zones)
    bp1x, bp1y = rotate_point(0, bottom_sensor_offset, q - math.radians(90 + bottom_sensor_angle))
    bp2x, bp2y = rotate_point(0, bottom_sensor_offset, q - math.radians(90 - bottom_sensor_angle))
    sensor1_pos = (x + bp1x, y + bp1y)
    sensor2_pos = (x + bp2x, y + bp2y)
    
    if is_sensor_in_danger_spot(*sensor1_pos, spots):
        visualizer.draw_point(*sensor1_pos, (255, 0, 0))
        world_map.mark_as_danger(*sensor1_pos)
    else:
        visualizer.draw_point(*sensor1_pos, (50, 120, 255))
        world_map.mark_as_safe(*sensor1_pos)
        
    if is_sensor_in_danger_spot(*sensor2_pos, spots):
        visualizer.draw_point(*sensor2_pos, (255, 0, 0))
        world_map.mark_as_danger(*sensor2_pos)
    else:
        visualizer.draw_point(*sensor2_pos, (50, 120, 255))
        world_map.mark_as_safe(*sensor2_pos)
    
    # Draw info
    visualizer.draw_text(f'Actual robot angle:        {math.degrees(q)}', (300, 20))
    visualizer.draw_text(f'Actual robot position:    {round_point(x, y)}', (300, 30))
    visualizer.draw_text(f'Estimated robot position: {round_point(-cx, -cy)}', (300, 40))
    visualizer.draw_text(f'Distance to goal:        {distance_to_goal}', (300, 50))

    visualizer.show()

    
    left_mult, right_mult = get_wheel_speeds(x, y, q, target_pos, sensors)

    # step simulation
    simulationstep(left_wheel_velocity * left_mult, right_wheel_velocity * right_mult)

    # check collision with arena walls
    if world.distance(Point(x, y)) < L / 2:
        file.write(str(x) + ", " + str(y) + ", " + str(cos(q) * 0.05) + ", " + str(sin(q) * 0.05) + "\n")
        print('Collision!', q, x, y)
        print(cnt, 'Steps')
        break

    if cnt % 50 == 0:
        file.write(str(x) + ", " + str(y) + ", " + str(cos(q) * 0.05) + ", " + str(sin(q) * 0.05) + "\n")

file.close()
