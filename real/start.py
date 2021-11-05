#!/usr/bin/python3

import os

# initialize asebamedulla in background and wait 0.3s to let asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")

from shared.movement import look_for_april_tag
import math
from statistics import mean
from shared.route_planner import turn_to_point

from shared.step import single_sim_step


from shared.map import Map
from shared.particle_filtering import compare_states, generate_random_particles, resample

from shared.util import angle_difference, calc_rectangle, euclidean_distance, find_robot_pos2, get_lidar_points, moving_average, rolling_average, sensor_readings_to_motor_speeds
import time
from time import sleep

from shared.state import L, R, W, H
import dbus
import dbus.mainloop.glib
from threading import Thread
import threading

import numpy as np

# sensors
from adafruit_rplidar import RPLidar
from picamera import PiCamera
import cv2
import apriltag

'''
Thymio class with sensing and driving functions
'''

def rect_to_distance(rect):
    _, (tx, ty), (bx, by), _ = rect
    return (by - ty)

def rect_to_angle(rect):
    (tl_x, tl_y), (tr_x, tr_y), (b_x, b_y), _ = rect
    width = tr_x - tl_x
    height = b_y - tr_y
    return height/width

def rect_to_relative_position(rect):
    (tl_x, _), (tr_x, _), _, _ = rect
    n1 = tl_x
    n2 = cam_size[0] - tr_x
    if abs(n1 - n2) < 100:
        return (1, 1)
        # Keep on truckin'
    else:
        if n1 > n2:
            return 1, 0.8
            # print('Turn right!')
        
        elif n2 > n1:
            return 0.8, 1
            # print('Turn left!')
    



class Thymio:
    def __init__(self):
        print("Starting robot")
        self.aseba = self.setup()

    def drive(self, left_wheel_speed, right_wheel_speed):
        # print("Left_wheel_speed: " + str(left_wheel_speed))
        # print("Right_wheel_speed: " + str(right_wheel_speed))

        left_wheel = left_wheel_speed
        right_wheel = right_wheel_speed
        print(left_wheel, right_wheel)

        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def get_motor_multipliers(self):
        sensor_values = [scan_data[360 - 30], scan_data[360 - 15], scan_data[0], scan_data[15], scan_data[30]]
        return sensor_readings_to_motor_speeds(sensor_values)


    '''
    Responsible for Thymio ground sensors
    Smoothing is implemented to account for noise
    (one step)
    '''

    def sens(self):
        global robot_position

        # Smoothing initialization
        ground_sensor_0 = [500, 500, 500, 500, 500]
        ground_sensor_1 = [500, 500, 500, 500, 500]

        print('Ground sensors on')

        while True:
            prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.reflected")
            smoothed_ground_0 = rolling_average(ground_sensor_0, prox_ground[0])
            smoothed_ground_1 = rolling_average(ground_sensor_1, prox_ground[1])

            # ground_sensor_0.insert(0, prox_ground[0])
            # del ground_sensor_0[5]
            # ground_sensor_1.insert(0, prox_ground[1])
            # del ground_sensor_1[5]

            # smoothed_ground_0 = mean(ground_sensor_0)
            # smoothed_ground_1 = mean(ground_sensor_1)
            try:
                if smoothed_ground_0 > 500 and smoothed_ground_1 > 500:
                    # print(robot_position)
                    world_map.mark_as_safe(*robot_position)
                else:
                    # print(robot_position)
                    world_map.mark_as_danger(*robot_position)
            except:
                pass

    '''
    Initialization of the Thymio
    '''
    def play_sound(self, num):
        self.aseba.SendEventName("sound.system", [num])

    def setup(self):
        print("Setting up")
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SessionBus()
        asebaNetworkObject = bus.get_object("ch.epfl.mobots.Aseba", "/")

        asebaNetwork = dbus.Interface(
            asebaNetworkObject, dbus_interface="ch.epfl.mobots.AsebaNetwork"
        )
        # load the file which is run on the thymio
        asebaNetwork.LoadScripts(
            "thympi.aesl", reply_handler=self.dbusError, error_handler=self.dbusError
        )

        # scanning_thread = Process(target=robot.drive, args=(200,200,))
        return asebaNetwork

    def stopAsebamedulla(self):
        os.system("pkill -n asebamedulla")

    def dbusReply(self):
        # dbus replys can be handled here.
        # Currently ignoring
        pass

    def dbusError(self, e):
        # dbus errors can be handled here.
        # Currently only the error is logged. Maybe interrupt the mainloop here
        print("dbus error: %s" % str(e))


# ----------------------- Lidar init -------------------------

PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

scan_data = [0] * 360  # store the lidar readings

import csv

center = (0, 0)
width_height = (0, 0)
rect_angle = 0

def estimate_current_angle(curr):
    curr = abs(curr)
    angles = [curr, curr + 90, curr + 180, curr + 270]
    
    min_diff = 10000
    correct_angle = 0
    for angle in angles:
        diff = abs(angle_difference(math.radians(robot_angle), math.radians(angle)))
        if diff < min_diff:
            min_diff = diff
            correct_angle = angle
    
    # print(curr)
    # print('Correct angle:', correct_angle)
    # print('robot_angle', robot_angle)
    return correct_angle


# Calculate the angle difference between two lidar scans
# in order to calculate robots actual angle
def calc_angle_delta(prev, curr):
    # Assumption: The robot does not spin more than 90 degrees between each lidar scan
    diff = curr - prev
    diff1 = diff + 90
    diff2 = diff - 90
    if abs(diff) < abs(diff1) and abs(diff) < abs(diff2):
        return diff
    elif abs(diff1) < abs(diff) and abs(diff1) < abs(diff):
        return diff1
    elif abs(diff2) < abs(diff) and abs(diff2) < abs(diff1):
        return diff2


angle_diff = 0

max_lidar_reading = math.sqrt(W**2 + H**2)

# Scan using lidar
def lidarScan():
    global scan_data, center, width_height, rect_angle, angle_diff, robot_angle, robot_position
    print("Starting background lidar scanning")
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            angle = round(angle + 180) % 360
            dist = (distance / 1000)
            if dist < 0.13 or dist > max_lidar_reading: # Discard incorrect reading
                print('Discarded', dist)
                scan_data[angle] = 0
            else:    
                scan_data[angle] = dist
        
        lidar_points = get_lidar_points(moving_average(scan_data), 0)

        center, width_height, rect_angle = calc_rectangle(lidar_points)
        new_angle = estimate_current_angle(rect_angle)

        # Only update, if the new angle is within 90 degrees of the previous angle
        if abs(angle_difference(new_angle, robot_angle)) < math.pi / 2:
            robot_angle = new_angle
        
        curr_pos = find_robot_pos2(center, math.radians(robot_angle))
        robot_position = curr_pos

# ----------------------- Camera init ------------------------

# initialize camera & time intervals for image capture
visible_april_tags = []
print('Camera starting up')
start_time = time.time()
camera = PiCamera()
cam_size = (640, 480)

camera.start_preview()
camera.resolution = cam_size
camera.framerate = 24

def takePicture():
    image = np.empty((480, 640, 3), dtype=np.uint8)
    camera.capture(image, 'bgr')
    flippedImage = cv2.flip(image, -1)
    gray = cv2.cvtColor(flippedImage, cv2.COLOR_BGR2GRAY)
    # print("Detecting AprilTags...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)

    return results

def get_best_tag(tags):
    tag_score = 10
    best_tag = None
    for tag in tags:
        angle = rect_to_angle(tag[7])
        diff = abs(1 - angle)
        if diff < tag_score:
            tag_score = diff
            best_tag = tag
    return best_tag

def tag_filter(tag):
    if tag[1] == 19: ## Keep buddy
        return True
    if rect_to_angle(tag[7]) > 2:
        return False
    return True

# Loop which takes images with a delay
def findtag():
    global visible_april_tags
    print('Looking for april tag')
    while True:
        visible_april_tags = list(filter(tag_filter, takePicture()))
        
        for tag in visible_april_tags:

            if tag[1] != 0:
                # for i, a in enumerate(tag):
                #     print(i, a)
                pass
                print("FOUND!", tag[1], rect_to_angle(tag[7]))
                # print("Tagsize", tag.tagsize)


# --------------------- init script end ----------------------

# ------------------ Main loop ------------------------------

world_map = Map(W, H, 10)

robot_speed = 600
buddy_speed = 200
MAX_ROBOT_SPEED = 600

is_buddy_picked_up = False
orientation = 0

particles = generate_random_particles(100)

left_tags = set([15, 0, 1])
up_tags = set([2, 3, 4, 5, 6])
right_tags = set([7, 8, 9])
down_tags = set([10, 11, 12, 13, 14])

from statistics import mean

robot_position = (0, 0)
robot_angle = 0
robot_pos_confidence = 0

looking_for_position = True

path_to_explore = []

scale = 0.11
dist = L * 1.8
for i in range(0, 7, 2):
    path_to_explore.append((W / 2 - dist, H / 2 - dist - scale * i))
    path_to_explore.append((-W / 2 + dist, H / 2 - dist - scale * i))
    path_to_explore.append((-W / 2 + dist, H / 2 - dist - scale * (i + 1)))
    path_to_explore.append((W / 2 - dist, H / 2 - dist - scale * (i + 1)))

path_index = 0

buddy_phase = 0
def look_for_buddy(robot):
    global buddy_phase
    if buddy_phase == 0:
        is_at_point = move_to_point(robot, (0, 0))
        if is_at_point:
            buddy_phase = 1
    elif buddy_phase == 1:
        robot.drive(0.3 * robot_speed, -0.3 * robot_speed)
        sleep(0.2)
        robot.stop()
        sleep(0.2)


def path_following(robot, speed=robot_speed):
    #### Path finding
    global path_index
    if path_index < len(path_to_explore):
        target_pos = path_to_explore[path_index]
        is_at_point = move_to_point(robot, target_pos, speed)

        if is_at_point:
            path_index += 1
            for _ in range(0, 10):
                print('#####Yehaaaw#####', path_index)
    
        print(robot_position, robot_angle, target_pos)
    
    return path_index < len(path_to_explore)

def move_to_point(robot, target_pos, speed=robot_speed):
    distance_to_goal = euclidean_distance(*target_pos, *robot_position)
    path_left_mult, path_right_mult = turn_to_point(*robot_position, math.radians(robot_angle), *target_pos, distance_to_goal)
    print(path_left_mult, path_right_mult)
    
    left_wheel_speed = speed * path_left_mult
    right_wheel_speed = speed * path_right_mult
    
    robot.drive(left_wheel_speed, right_wheel_speed)
    return distance_to_goal < 0.04

def main():
    global path_to_explore, path_index, robot_speed, particles, is_buddy_picked_up, orientation, robot_position, robot_angle, robot_pos_confidence, looking_for_position
    try:
        robot = Thymio()

        # Start lidar scanning
        scanner_thread = threading.Thread(target=lidarScan)
        scanner_thread.daemon = True
        scanner_thread.start()

        # Start ground sensing
        sensing_thread = Thread(target=robot.sens)
        sensing_thread.daemon = True
        sensing_thread.start()

        # Start april tag sensing
        sensing_thread = Thread(target=findtag)
        sensing_thread.daemon = True
        sensing_thread.start()
        
        # make it drive and avoid walls
        print('Driving!')

        while True:
            # os.system('cls' if os.name == 'nt' else 'clear')
            buddy_tag = None
            for tag in visible_april_tags:
                if tag[1] == 19:
                    buddy_tag = tag
                
            #### Positioning
            w, h = width_height

            tag = get_best_tag(visible_april_tags)
            if tag is not None:
                if w > h:
                    if tag[1] in left_tags or tag[1] in up_tags:
                        orientation = 0
                    elif tag[1] in right_tags or tag[1] in down_tags:
                        orientation = 180
                else:
                    if tag[1] in left_tags or tag[1] in down_tags:
                        orientation = 270
                    elif tag[1] in right_tags or tag[1] in up_tags:
                        orientation = 90
            
                robot_angle = orientation + abs(rect_angle)
                curr_pos = find_robot_pos2(center, math.radians(robot_angle))
                robot_position = curr_pos
                print('######April update!', orientation)
                
            if not is_buddy_picked_up:
                if buddy_tag != None:
                    buddy_distance = rect_to_distance(buddy_tag[7])
                    if buddy_distance < 163:
                        left_multiplier, right_multiplier = rect_to_relative_position(buddy_tag[7])
                        left_wheel_speed = buddy_speed * left_multiplier
                        right_wheel_speed = buddy_speed * right_multiplier
                        robot.drive(left_wheel_speed, right_wheel_speed)
                    else:
                        if not is_buddy_picked_up:
                            robot.play_sound(7)
                            print("I PICKED IT UP!")
                            goal = world_map.map_cord_to_real(2, 2)
                            path_to_explore = world_map.find_safe_path(robot_position, goal)
                            print(path_to_explore)
                            path_index = 0
                            is_buddy_picked_up = True
                            sleep(0.5)
                
                if buddy_tag == None:
                    if len(visible_april_tags) == 0 and looking_for_position:
                        # Move back if it is too close to the wall!
                        left_multiplier1, right_multiplier1 = robot.get_motor_multipliers()
                        left_multiplier, right_multiplier = look_for_april_tag()
                        left_wheel_speed = robot_speed * left_multiplier * left_multiplier1
                        right_wheel_speed = robot_speed * right_multiplier * right_multiplier1
                        robot.drive(left_wheel_speed, right_wheel_speed)
                        sleep(0.1)
                        robot.stop()
                        sleep(0.4)
                    else:
                        looking_for_position = False
                        #### Path finding
                        is_following = path_following(robot)
                        if not is_following:
                            look_for_buddy(robot)
            else:
                is_following = path_following(robot, 300)
                
            world_map.pretty_print()
            
            # with open('particles.csv', 'a') as f:
            #     writer = csv.writer(f)
            #     writer.writerow(particles)

            sleep(0.1)
    except KeyboardInterrupt:
        tear_down(robot)
    except Exception as e:
        tear_down(robot)
        raise e


def tear_down(robot):
    print("Stopping robot")
    robot.stop()
    sleep(1)
    lidar.stop()
    lidar.disconnect()
    os.system("pkill -n asebamedulla")
    print("asebamedulla killed")
    camera.stop_preview()


# ------------------- Main loop end ------------------------

if __name__ == '__main__' or __name__ == 'real.start':
    main()

# -------------------
