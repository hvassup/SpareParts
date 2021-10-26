import math

from numpy import sin, cos
from shapely.geometry import Point
from shared.map import Map
from shared.movement import get_wheel_speeds
from shared.particle_filtering import compare_states, generate_random_particles, resample
from shared.route_planner import angle_to_point
from shared.step import single_sim_step
from simulation.bottom_sensor import is_sensor_in_danger_spot
from simulation.danger_zones import generate_random_danger_spots

from simulation.sensor_sim import distance_to_sensor_reading, get_lidar, get_sensor_distance
from simulation.visualization.pygame_visualizer import PyGameVisualizer, scale_point

from shared.util import calc_rectangle, euclidean_distance, get_lidar_points, is_point_inside_rectangle, rand, rotate_point, round_point

from shared.state import W, H, L, world, robot_timestep, simulation_timestep

def get_lidar_reading(resolution=360):
    return get_lidar(x, y, q, resolution)

def get_front_sensor(angle):
    return distance_to_sensor_reading(get_sensor_distance(x, y, q, angle))

# Variables

x = 0.0  # robot position in meters - x direction - positive to the right
y = 0.0  # robot position in meters - y direction - positive up
q = 0.0  # robot heading with respect to x-axis in radians

left_wheel_velocity = 2   # robot left wheel velocity in radians/s
right_wheel_velocity = 2  # robot right wheel velocity in radians/s


# Kinematic model

# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot
def simulationstep(_left_wheel_velocity, _right_wheel_velocity):
    global x, y, q, particles

    for step in range(int(robot_timestep / simulation_timestep)):  # step model time/timestep times
        x, y, q = single_sim_step(x, y, q, _left_wheel_velocity, _right_wheel_velocity)
    
        particles = list(map(lambda p: single_sim_step(*p, _left_wheel_velocity, _right_wheel_velocity), particles))

# spots = generate_random_danger_spots(W, H)
spots = []

visualizer = PyGameVisualizer()
# visualizer = MatPlotVisualizer()

file = open("trajectory.dat", "w")
turn_counter = 0

buddy_pos = (rand(W), rand(H))

path_to_explore = [buddy_pos]
path_index = 0

world_map = Map(W, H, 3)

for _y in range(0, world_map.map.shape[1]):
    for _x in range(0, world_map.map.shape[0]):
        for zone in spots:
            real_point = _x * world_map.RESOLUTION - world_map.WIDTH / 2, _y * world_map.RESOLUTION - world_map.HEIGHT / 2
            if is_point_inside_rectangle(*real_point, *zone):
                world_map.mark_as_danger(*real_point)

april_tags = [(W / 2, 0), (W / 2, H / 3),
(W * 2 / 5, H / 2), (W / 5, H / 2), (0, H / 2), (-W / 5, H / 2), (-W * 2 / 5, H / 2),
(-W / 2, H / 3), (-W / 2, 0), (-W / 2, -H / 3),
(-W * 2 / 5, -H / 2), (-W / 5, -H / 2), (0, -H / 2), (W / 5, -H / 2), (W * 2 / 5, -H / 2), 
(W / 2, -H / 3)]

camera_range = 0.50
camera_fov = 25

bottom_sensor_offset = L / 2
bottom_sensor_angle = 10

is_buddy_picked_up = False

return_point = (rand(W), rand(H))

particles = generate_random_particles(20)

# Simulation loop
for cnt in range(1, 5000):
    sensors = [get_front_sensor(30), get_front_sensor(15), get_front_sensor(0), get_front_sensor(-15), get_front_sensor(30)]
    
    # Init visualization
    visualizer.clear()

    # Draw map
    for safe_spot in world_map.safe_spots:
        res = world_map.RESOLUTION
        visualizer.draw_rectangle(safe_spot, (res / 100, res / 100), (87, 184, 255))
    # for _y in range(0, map.map.shape[1]):
    #     for _x in range(0, map.map.shape[0]):
    #         pos = (_x * res - map.WIDTH / 2, _y * res - map.HEIGHT / 2)
    #         if map.is_danger(*pos):
    #             pass # visualizer.draw_rectangle(pos, (res, res), (254, 28, 28))
    #         else:
    #             visualizer.draw_rectangle(pos, (res, res), (87, 184, 255))

    # Draw lidar
    lidar_reading = get_lidar_reading(50)
    lidar_points = get_lidar_points(lidar_reading, q)
    center, width_height, _angle = calc_rectangle(lidar_points)
    cx, cy = center 
    visualizer.draw_points(lidar_points, (120, 120, 120), x, y)

    # Draw camera field of view
    cp1x, cp1y = rotate_point(0, camera_range, q - math.radians(90 + camera_fov))
    cp2x, cp2y = rotate_point(0, camera_range, q - math.radians(90 - camera_fov))
    visualizer.draw_line(x, y, x + cp1x, y + cp1y, (50, 0, 255))
    visualizer.draw_line(x, y, x + cp2x, y + cp2y, (50, 0, 255))

    # Draw april tags
    for i, pos in enumerate(april_tags):
        visualizer.draw_point(*pos, (255, 255, 0), 0.02)
        visualizer.draw_text(str(i), scale_point(*pos))
        if euclidean_distance(*pos, x, y) < camera_range and abs(math.degrees(angle_to_point(x, y, q, *pos))) < camera_fov:
            visualizer.draw_line(x, y, *pos, (255, 0 ,0))
            # print(f'I can see {i}')

    # Draw buddy
    if not is_buddy_picked_up:
        visualizer.draw_point(*buddy_pos, (255, 192, 203), 0.05)
        if euclidean_distance(*buddy_pos, x, y) < L:
            is_buddy_picked_up = True
            path_to_explore = world_map.find_safe_path(buddy_pos, return_point)

    # Draw danger spots
    visualizer.draw_rectangles(spots, (120, 0, 0))
    
    # visualizer.draw_point(-cx, -cy, (0, 0, 255))
    # Draw robot
    visualizer.draw_point(x, y, (0, 255, 0), L)

    # Path finding
    target_pos = path_to_explore[path_index]
    
    distance_to_goal = euclidean_distance(*target_pos, x, y)
    
    if distance_to_goal < L / 10:
        path_index += 1

    visualizer.draw_points(path_to_explore, (0, 0, 0), 0, 0)
    visualizer.draw_point(*target_pos, (0, 255, 255))
    visualizer.draw_line(*target_pos, x, y, (255, 0, 255))

    # Simulate bottom sensors (They can only see danger zones)
    bp1x, bp1y = rotate_point(0, bottom_sensor_offset, q - math.radians(90 + bottom_sensor_angle))
    bp2x, bp2y = rotate_point(0, bottom_sensor_offset, q - math.radians(90 - bottom_sensor_angle))
    sensor1_pos = (x + bp1x, y + bp1y)
    sensor2_pos = (x + bp2x, y + bp2y)

    is_bottom1_sensor_danger = is_sensor_in_danger_spot(*sensor1_pos, spots)
    is_bottom2_sensor_danger = is_sensor_in_danger_spot(*sensor2_pos, spots)
    
    if is_bottom1_sensor_danger:
        visualizer.draw_point(*sensor1_pos, (255, 0, 0))
        world_map.mark_as_danger(*sensor1_pos)
    else:
        visualizer.draw_point(*sensor1_pos, (50, 120, 255))
        world_map.mark_as_safe(*sensor1_pos)
        # if return_point == None:
        #     return_point = sensor1_pos
        
    if is_bottom2_sensor_danger:
        visualizer.draw_point(*sensor2_pos, (255, 0, 0))
        world_map.mark_as_danger(*sensor2_pos)
    else:
        visualizer.draw_point(*sensor2_pos, (50, 120, 255))
        world_map.mark_as_safe(*sensor2_pos)
    
    weights = [compare_states(p, lidar_reading) for p in particles]
    print('min:', min(weights), 'max:', max(weights))

    m = max(weights)
    for i, p in enumerate(particles):
        visualizer.draw_point(p[0], p[1], (150, 0, 150), weights[i] / m / 80)
    
    particles = resample(particles, weights)

    # Draw info
    visualizer.draw_text(f'Actual robot angle:        {math.degrees(q)}', (300, 20))
    visualizer.draw_text(f'Actual robot position:    {round_point(x, y)}', (300, 30))
    visualizer.draw_text(f'Estimated robot position: {round_point(-cx, -cy)}', (300, 40))
    visualizer.draw_text(f'Distance to goal:        {distance_to_goal}', (300, 50))
    visualizer.draw_text(f'Buddy pos:        {buddy_pos}', (300, 60))
    visualizer.draw_text(f'Start pos:        {return_point}', (300, 70))

    visualizer.show()

    
    left_mult, right_mult = get_wheel_speeds(x, y, q, target_pos, sensors, is_bottom1_sensor_danger, is_bottom2_sensor_danger)

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
