#!/usr/bin/python3

import os

from shared.step import single_sim_step

# initialize asebamedulla in background and wait 0.3s to let asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")

from shared.map import Map
from shared.particle_filtering import compare_states, generate_random_particles, resample

from shared.util import moving_average, rolling_average
import time
from time import sleep

from shared.state import W, H
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


class Thymio:
    def __init__(self):
        print("Starting robot")
        self.aseba = self.setup()

    def drive(self, left_wheel_speed, right_wheel_speed):
        # print("Left_wheel_speed: " + str(left_wheel_speed))
        # print("Right_wheel_speed: " + str(right_wheel_speed))

        left_wheel = left_wheel_speed
        right_wheel = right_wheel_speed

        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def get_motor_multipliers(self):
        if scan_data[0] < 0.15:
            return -1, 1
        else:
            return 1, 1

    '''
    Responsible for Thymio ground sensors
    Smoothing is implemented to account for noise
    (one step)
    '''

    def sens(self):

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

            if smoothed_ground_0 > 500 and smoothed_ground_1 > 500:
                # White
                pass
            else:
                # Black
                pass

    '''
    Initialization of the Thymio
    '''

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


# Scan using lidar
def lidarScan():
    print("Starting background lidar scanning")
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            angle = round(angle + 180) % 360
            scan_data[angle] = (distance / 1000)
        scan_data = moving_average(scan_data)

        with open('lidar.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow(scan_data)


# ----------------------- Camera init ------------------------

# initialize camera & time intervals for image capture
visible_april_tags = []
print('Camera starting up')
start_time = time.time()
camera = PiCamera()


def takePicture():
    camera.start_preview()

    camera.resolution = (640, 480)
    camera.framerate = 24

    image = np.empty((480, 640, 3), dtype=np.uint8)
    camera.capture(image, 'bgr')
    flippedImage = cv2.flip(image, -1)
    gray = cv2.cvtColor(flippedImage, cv2.COLOR_BGR2GRAY)
    camera.stop_preview()
    # print("Detecting AprilTags...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)

    # print("{} total AprilTags detected".format(len(results)))
    return results


# Loop which takes images with a delay
def findtag():
    global visible_april_tags
    print('Looking for april tag')
    while True:
        sleep(0.1)
        visible_april_tags = takePicture()
        for tag in visible_april_tags:

            if tag[1] != 0:
                # for a in tag:
                #     print(a)
                print("FOUND!", tag[1])
                # print("Tagsize", tag.tagsize)


# --------------------- init script end ----------------------

# ------------------ Main loop ------------------------------

world_map = Map(W, H, 5)

robot_speed = 600
MAX_ROBOT_SPEED = 600

particles = generate_random_particles(100)


def main():
    global particles
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
        global robot_speed
        while True:
            print(robot_speed)
            left_multiplier, right_multiplier = robot.get_motor_multipliers()
            left_wheel_speed = robot_speed * left_multiplier
            right_wheel_speed = robot_speed * right_multiplier
            robot.drive(left_wheel_speed, right_wheel_speed)
            weights = []
            for p in particles:
                weights.append(compare_states(p, scan_data, visible_april_tags))
            particles = resample(particles, weights)

            l_radians_per_second = left_wheel_speed / MAX_ROBOT_SPEED * 9.53
            r_radians_per_second = right_wheel_speed / MAX_ROBOT_SPEED * 9.53

            particles = list(map(lambda p: single_sim_step(*p, l_radians_per_second, r_radians_per_second), particles))

            with open('particles.csv', 'a') as f:
                writer = csv.writer(f)
                writer.writerow(particles)

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


# ------------------- Main loop end ------------------------

if __name__ == '__main__' or __name__ == 'real.start':
    main()

# -------------------
