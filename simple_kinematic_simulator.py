import math
import shapely
from shapely.geometry import LinearRing, LineString, Point
from numpy import sin, cos, pi, sqrt
from random import random

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

import matplotlib.pyplot as plt

# Variables 
###########

x = 0.0   # robot position in meters - x direction - positive to the right 
y = 0.0   # robot position in meters - y direction - positive up
q = 0.0   # robot heading with respect to x-axis in radians 

left_wheel_velocity = random()   # robot left wheel velocity in radians/s
right_wheel_velocity = random()  # robot right wheel velocity in radians/s

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

def sensor_readings_to_motor_speeds(sensor1, sensor2, sensor3, sensor4, sensor5):
    sensor1 /= 5020
    sensor2 /= 5020
    sensor3 /= 5020
    sensor4 /= 5020
    sensor5 /= 5020
    #sensor1, sensor2, sensor3, sensor4, sensor5 = list(map(lambda v: v/5020, (1,2,3,4,5)))
    
    return 1 - (sensor4 + sensor5), 1 - (sensor1 + sensor2)


def distance_to_sensor_reading(distance):
    max_dist = 0.1
    max_reading = 5020
    if distance > max_dist:
        return 0
    return (1 - (distance / max_dist)) * max_reading
    
    

def get_sensor_distance(angle):
    angle = angle / 180 * math.pi # Convert to radians
    #simple single-ray sensor
    ray = LineString([(x, y), (-(y + sin(q + angle) * 2 * H), x + cos(q + angle) * 2 * W) ])  # a line from robot to a point outside arena in direction of q
    s = world.intersection(ray)
    distance = sqrt((s.x-x)**2+(s.y-y)**2) - 0.05                    # distance to wall

    return distance# distance_to_sensor_reading(distance)

# Simulation loop
#################
file = open("trajectory.dat", "w")

for cnt in range(5000):
    #simple controller - change direction of wheels every 10 seconds (100*robot_timestep) unless close to wall then turn on spot
    # sensor1 = get_sensor_distance(30)
    # sensor2 = get_sensor_distance(15)
    sensor3 = get_sensor_distance(0)
    # sensor4 = get_sensor_distance(-15)
    # sensor5 = get_sensor_distance(-30)
    

    #print(sensor1, sensor2, sensor3, sensor4, sensor5)
    left_mult, right_mult = (1, 1) # sensor_readings_to_motor_speeds(sensor1, sensor2, sensor3, sensor4, sensor5)
    print(sensor3)
    # print(left_mult, right_mult)
    if cnt%1000==0:
        left_wheel_velocity = random()
        right_wheel_velocity = random()
        
    #step simulation
    simulationstep(left_wheel_velocity * left_mult, right_wheel_velocity * right_mult)

    #check collision with arena walls
    if (world.distance(Point(x,y))<L/2):
        print('Collision!', q, x, y)
        ray = LineString([(x, y), (x+cos(q)*2*W,-(y+sin(q)*2*H)) ])  # a line from robot to a point outside arena in direction of q
        plt.plot(*world.xy)
        plt.plot(*ray.xy)
        plt.show()
        break
        
    if cnt%50==0:
        file.write( str(x) + ", " + str(y) + ", " + str(cos(q)*0.05) + ", " + str(sin(q)*0.05) + "\n")

file.close()
    
import visualizer

visualizer.visualize()