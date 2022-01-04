#!/usr/bin/python3
import os
from random import random

# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")

from time import sleep
from enums import Action, State

from util import MajorityVoter, ValueSmoother

import dbus
import dbus.mainloop.glib

R = 0.043  # radius of wheels in meters
L = 0.092  # distance between wheels in meters

class Thymio:
    def __init__(self, is_seeker=False):
        self.aseba = self.setup()
        self.is_seeker = is_seeker
        self.speed = 500
        self.left_sensor_smoother = ValueSmoother(3, 900)
        self.right_sensor_smoother = ValueSmoother(3, 900)
        self.sensor_state = State.NoSensors
        self.left_is_gray = MajorityVoter(11, False)
        self.right_is_gray = MajorityVoter(11, False)
        self.random_counter = 0
        self.random_left = 0
        self.random_right = 0

    def drive(self, left_wheel_speed, right_wheel_speed):
        # print("Left_wheel_speed: " + str(left_wheel_speed))
        # print("Right_wheel_speed: " + str(right_wheel_speed))
        
        left_wheel = self.speed * left_wheel_speed
        right_wheel = self.speed * right_wheel_speed
        
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def sens_front(self):
        prox_horizontal = self.aseba.GetVariable("thymio-II", "prox.horizontal")
        return prox_horizontal[0] + prox_horizontal[1] + prox_horizontal[2] + prox_horizontal[3] + prox_horizontal[4]
    
    def sense_bottom(self):
        # prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.ambiant")
        # print(f'Ambiant,{prox_ground[0]},{prox_ground[1]}')
        
        prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.reflected")
        # print(f'Reflected,{prox_ground[0]},{prox_ground[1]}')
        
        # prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.delta")
        # print(f'Delta,{prox_ground[0]},{prox_ground[1]}')

        return prox_ground
    
    def clear_gray(self):
        self.left_is_gray.clear(False)
        self.right_is_gray.clear(False)
    
    def set_color(self, color):
        self.aseba.SendEventName("leds.top", color)
        self.aseba.SendEventName("leds.bottom.right", color)
        self.aseba.SendEventName("leds.bottom.left", color)
    
    def turn_red(self):
        self.set_color([32, 0, 0])

    def turn_yellow(self):
        self.set_color([32, 32, 0])

    def turn_blue(self):
        self.set_color([0, 0, 32])

    def turn_green(self):
        self.set_color([0, 32, 0])

    def turn_purple(self):
        self.set_color([32, 0, 32])

    def sendInformation(self, number):
        self.aseba.SendEventName("prox.comm.tx", [number])
    
    def becomeSeeker(self):
        self.seeker = True
        self.sendInformation(1)
        
    def becomeAvoider(self):
        self.seeker = False
        self.sendInformation(2)

    def receiveInformation(self):
        rx = self.aseba.GetVariable("thymio-II", "prox.comm.rx")
        return rx[0]
    
    def disableComms(self):
        self.aseba.SendEventName("prox.comm.enable", [0])

    def enable_comms(self):
        self.aseba.SendEventName("prox.comm.enable", [1])
    
    def disable_sensor_leds(self):
        self.aseba.SendEventName("leds.temperature", [0])
        self.aseba.SendEventName("leds.circle", [0])
        self.aseba.SendEventName("leds.prox.h", [0, 0, 0, 0, 0])
        self.aseba.SendEventName("leds.prox.v", [0, 0])
        self.aseba.SendEventName("leds.buttons", [0])
        self.aseba.SendEventName("leds.sound", [0])
        self.aseba.SendEventName("leds.rc", [0])
    
    def restart_comms(self):
        if self.receiveInformation() != 0:
            self.disableComms()
            self.enable_comms()
    
    def play_sound(self, sound):
        self.aseba.SendEventName("sound.system", [sound])

    def get_state(self):
        left_sensor, right_sensor = self.sense_bottom()
        
        left_sensor_reading = self.left_sensor_smoother.update_value(left_sensor)
        right_sensor_reading = self.right_sensor_smoother.update_value(right_sensor)
        is_left_black = left_sensor_reading < 700
        is_right_black = right_sensor_reading < 700
        if is_left_black and is_right_black:
            self.sensor_state = State.BothSensors
        elif is_left_black and not is_right_black:
            self.sensor_state = State.LeftSensor
        elif not is_left_black and is_right_black:
            self.sensor_state = State.RightSensor
        else:
            self.sensor_state = State.NoSensors

        return self.sensor_state
    
    def is_gray(self):
        left_sensor, right_sensor = self.sense_bottom()
        
        is_left = 930 > left_sensor > 700
        is_right = 930 > right_sensor > 700

        is_left_gray = self.left_is_gray.update_value(is_left)
        is_right_gray = self.right_is_gray.update_value(is_right)

        is_gray = is_left_gray and is_right_gray

        # if is_gray:
        #     print('Saw gray!')
        #     print(left_sensor, right_sensor)

        return is_gray
        

        
    def perform_action(self, action):
        if action == Action.Forward:
            self.go_forward()
            sleep(0.05)
        elif action == Action.Left:
            self.go_left()
            sleep(0.1)
        elif action == Action.Right:
            self.go_right()
            sleep(0.1)
        elif action == Action.Back:
            self.go_backward()
            sleep(0.05)

    def turn_degrees(self, deg):
        sleep_time = abs((deg / self.speed) * 3)
        if deg < 0:
            self.go_left()
        elif deg > 0:
            self.go_right()
        sleep(sleep_time)
    
    def go_left(self, speed=1):
        self.drive(-1 * speed, 1 * speed)

    def go_forward(self, speed=1):
        if not self.is_seeker:
            self.random_counter += 1
            if self.random_counter % 5 == 0:
                self.random_left, self.random_right = (random() * 0.4), (random() * 0.4)
            self.drive(1 * speed - self.random_left, 1 * speed - self.random_right)
        else:
            self.drive(1 * speed, 1 * speed)

    def go_right(self, speed=1):
        self.drive(1 * speed, -1 * speed)

    def go_backward(self, speed=1):
        if random() > 0.5:
            self.drive(-1 * speed * 0.3, -1 * speed)
        else:
            self.drive(-1 * speed, -1 * speed * 0.3)

    def go_stop(self):
        self.stop()
        self.restart_comms()

    ############## Bus and aseba setup ######################################
    def setup(self):
        # print("Setting up")
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

        # Enable communications
        asebaNetwork.SendEventName("prox.comm.enable", [1])

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


# #this enables the prox.com communication channels
# asebaNetwork.SendEventName( "prox.comm.enable", [1]) hello
# #This enables the prox.comm rx value to zero, gets overwritten when receiving a value
# asebaNetwork.SendEventName("prox.comm.rx",[0])
# def sendInformation(number):
#     asebaNetwork.SendEventName("prox.comm.tx", [number])
# def receiveInformation():
#     rx = asebaNetwork.GetVariable("thymio-II", "prox.comm.rx")
#     print(rx[0])
