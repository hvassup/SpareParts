import math
import shapely
from shapely.geometry import LinearRing, LineString, Point
from numpy import sin, cos, pi, sqrt
from random import random

from visualizer import plot_lidar
# A prototype simulation of a differential-drive robot with one sensor

# Constants
###########
R = 0.043  # radius of wheels in meters
L = 0.092  # distance between wheels in meters

W = 2.0  # width of arena
H = 2.0  # height of arena

robot_timestep = 0.1        # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch..)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W/2,H/2),(-W/2,H/2),(-W/2,-H/2),(W/2,-H/2)])
obstacle = LinearRing([(W/4,H/4),(-W/4,H/4),(-W/4,-H/4),(W/4,-H/4)])

import matplotlib.pyplot as plt

# Variables 
###########

x = W/3   # robot position in meters - x direction - positive to the right 
y = 0.0   # robot position in meters - y direction - positive up
q = math.pi   # robot heading with respect to x-axis in radians 

left_wheel_velocity = 1   # robot left wheel velocity in radians/s
right_wheel_velocity = 1  # robot right wheel velocity in radians/s

# Kinematic model
#################
# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot - don't worry just use it
def simulationstep(_left_wheel_velocity, _right_wheel_velocity):
    global x, y, q

    for step in range(int(robot_timestep/simulation_timestep)):     #step model time/timestep times
        v_x = cos(q)*(R*_left_wheel_velocity/2 + R*_right_wheel_velocity/2)
        v_y = sin(q)*(R*_left_wheel_velocity/2 + R*_right_wheel_velocity/2)
        omega = (R*_right_wheel_velocity - R*_left_wheel_velocity)/(2*L)
    
        x += v_x * simulation_timestep
        y += v_y * simulation_timestep
        q += omega * simulation_timestep


def distance_to_sensor_reading(distance):
    max_dist = 0.1
    max_reading = 5020
    
    if distance > max_dist:
        return 0
    return (1 - (distance / max_dist)) * max_reading

def get_sensor_distance(angle):
    angle = angle / 180 * math.pi # Convert to radians
    #simple single-ray sensor
    ray = LineString([(x, y), (x + cos(q + angle) * 2 * W, (y + sin(q + angle) * 2 * H)) ])  # a line from robot to a point outside arena in direction of q
    s = world.union(obstacle).intersection(ray)
    
    if type(s) is shapely.geometry.multipoint.MultiPoint:
        s = s[-1]

    distance = sqrt((s.x-x)**2+(s.y-y)**2) - 0.05                    # distance to wall

    return distance

def get_lidar(resolution=360):
    return [get_sensor_distance(i) for i in range(0, resolution)]


# Simulation loop
#################
file = open("trajectory.dat", "w")

for cnt in range(5000):
    #simple controller - change direction of wheels every 10 seconds (100*robot_timestep) unless close to wall then turn on spot
    sensor1 = distance_to_sensor_reading(get_sensor_distance(30))
    sensor2 = distance_to_sensor_reading(get_sensor_distance(15))
    sensor3 = distance_to_sensor_reading(get_sensor_distance(0))
    sensor4 = distance_to_sensor_reading(get_sensor_distance(-15))
    sensor5 = distance_to_sensor_reading(get_sensor_distance(-30))

    print(get_lidar())
    plot_lidar(0, 0, get_lidar())
    exit()

    ## print(sensor1, sensor2, sensor3, sensor4, sensor5)
    left_mult, right_mult = sensor_readings_to_motor_speeds(sensor1, sensor2, sensor3, sensor4, sensor5)
    print(left_mult, right_mult, sensor3, q * 180/math.pi)
    # if cnt%1000==0:
    #     left_wheel_velocity = random()
    #     right_wheel_velocity = random()
    # left_wheel_velocity *= left_mult
    # right_wheel_velocity *= right_mult
    #step simulation
    simulationstep(left_wheel_velocity * left_mult, right_wheel_velocity * right_mult)

    #check collision with arena walls
    if (world.distance(Point(x,y))<L/2):
        file.write( str(x) + ", " + str(y) + ", " + str(cos(q)*0.05) + ", " + str(sin(q)*0.05) + "\n")
        print('Collision!', q, x, y)
        print(cnt, 'Steps')
        break
        
    if cnt % 50 == 0:
        file.write( str(x) + ", " + str(y) + ", " + str(cos(q)*0.05) + ", " + str(sin(q)*0.05) + "\n")
    
file.close()
    
import visualizer

visualizer.visualize()