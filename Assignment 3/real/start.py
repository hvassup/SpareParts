#!/usr/bin/python3

import os
from shared.action_states import get_current_state

# initialize asebamedulla in background and wait 0.3s to let asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")


from shared.enums import State, Action
from shared.q_learning import get_next_action, update_q_table

from time import sleep

import dbus
import dbus.mainloop.glib

class Thymio:
    def __init__(self):
        print("Starting robot")
        self.aseba = self.setup()
        self.speed = 300

    def drive(self, left_wheel_speed, right_wheel_speed):
        left_wheel = left_wheel_speed * self.speed
        right_wheel = right_wheel_speed * self.speed 
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])
    
    def go_left(self):
        self.drive(-1, 1)
        sleep(1)
        self.stop()

    def go_forward(self):
        self.drive(1, 1)
        sleep(1)
        self.stop()

    def go_right(self):
        self.drive(1, -1)
        sleep(1)
        self.stop()

    def go_backward(self):
        self.drive(-1, -1)
        sleep(1)
        self.stop()
    
    def get_front_sensors(self):
        return self.aseba.GetVariable("thymio-II", "prox.horizontal")[:5]
    
    def get_state(self):
        return get_current_state(self.get_front_sensors())
    
    def perform_action(self, action):
        if action == Action.Forward:
            self.go_forward()
        elif action == Action.Left:
            self.go_left()
        elif action == Action.Right:
            self.go_right()
        elif action == Action.Back:
            self.go_backward()
    
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



# States
# OL - Object Left
# Anything on the two left sensors and nothing on the middle
# OR - Object Right
# Anything on the two right sensors and nothing on the middle
# OR - Object Right
# Anything on the front sensor
# NO - Object Right
# Nothing on any sensors


def main():
    try:
        robot = Thymio()
        sleep(1)

        while True:
            current_state = robot.get_state()
            action_to_perform = Action(get_next_action(current_state))
            robot.perform_action(action_to_perform)
            next_state = robot.get_state()
            print('current_state:', current_state, 'action_to_perform:', action_to_perform, 'next_state:', next_state)
            update_q_table(current_state, action_to_perform, next_state)

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
    os.system("pkill -n asebamedulla")
    print("asebamedulla killed")


# ------------------- Main loop end ------------------------

print(__name__)
if __name__ == 'real.start':
    main()

# -------------------
