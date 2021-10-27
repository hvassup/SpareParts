#!/usr/bin/python3

import os
from shared.util import sensor_readings_to_motor_speeds
import time
from time import sleep

# initialize asebamedulla in background and wait 0.3s to let asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")

import dbus
import dbus.mainloop.glib
from threading import Thread
import threading

from math import cos, sin, pi, floor
import numpy as np
from statistics import mean

# sensors
from adafruit_rplidar import RPLidar
from picamera import PiCamera
import cv2
import apriltag

print("Starting robot")

'''
Thymio class with sensing and driving functions
'''
class Thymio:
    def __init__(self):
        self.aseba = self.setup()

    def drive(self, left_wheel_speed, right_wheel_speed):
        print("Left_wheel_speed: " + str(left_wheel_speed))
        print("Right_wheel_speed: " + str(right_wheel_speed))

        left_wheel = left_wheel_speed
        right_wheel = right_wheel_speed

        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def get_motor_multipliers(self):
        prox_horizontal = self.aseba.GetVariable("thymio-II", "prox.horizontal")
        return sensor_readings_to_motor_speeds(prox_horizontal)
    
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
        
        prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.reflected")
        
        ground_sensor_0.insert(0, prox_ground[0])
        del ground_sensor_0[5]
        ground_sensor_1.insert(0, prox_ground[1])
        del ground_sensor_1[5]

        smoothed_ground_0 = mean(ground_sensor_0)
        smoothed_ground_1 = mean(ground_sensor_1)

        if smoothed_ground_0 > 500 and smoothed_ground_1 > 500:
            print("White")
        else:
            print("Black")

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

#----------------------- Lidar init -------------------------

PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

scan_data = [0]*360 # store the lidar readings

# Scan using lidar
def lidarScan():
    print("Starting background lidar scanning")
    for scan in lidar.iter_scans():
        if(exit_now):
            return
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance

def testLidar():
    print(scan_data)

#----------------------- Camera init ------------------------

# initialize camera & time intervals for image capture
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
    if results == []:
        return (0)
    else:
        return (results[0][1])

# Loop which takes images with a delay
def findtag():
    print('Looking for april tag')
    while True:
        tag = 0
        rounded = round(time.time() - start_time, 2)
        if rounded % 1 == 0:
            tag = takePicture()

        if tag != 0:
            print("FOUND!")

#--------------------- init script end ----------------------

# ------------------ Main loop ------------------------------

def main():
    robot = Thymio()

    sensing_thread = Thread(target=robot.sens)
    sensing_thread.daemon = True
    sensing_thread.start()

    # Start lidar scanning
    scanner_thread = threading.Thread(target=lidarScan)
    scanner_thread.daemon = True
    scanner_thread.start()
    
    # make it drive and avoid walls
    while True:
        print('Driving!')
        left_multiplier, right_multiplier = robot.get_motor_multipliers()
        robot.drive(200 * left_multiplier, 200 * right_multiplier)
        sleep(0.01)
    robot.stop()


# ------------------- Main loop end ------------------------

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Stopping robot")
        exit_now = True
        sleep(1)
        lidar.stop()
        lidar.disconnect()
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")

# -------------------