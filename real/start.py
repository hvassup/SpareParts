#!/usr/bin/python3
import os

# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
from shared.util import sensor_readings_to_motor_speeds

os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")
from time import sleep
import dbus
import dbus.mainloop.glib
from threading import Thread



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
        return sensor_readings_to_motor_speeds(prox_horizontal[0], prox_horizontal[1], prox_horizontal[2],
                                               prox_horizontal[3], prox_horizontal[4])

    # method for sensing
    # smoothing implemented for both the ground sensors and horizontal sensors
    def sens(self):

        front_sensor_0 = [0, 0, 0, 0, 0]
        front_sensor_1 = [0, 0, 0, 0, 0]
        front_sensor_2 = [0, 0, 0, 0, 0]
        front_sensor_3 = [0, 0, 0, 0, 0]
        front_sensor_4 = [0, 0, 0, 0, 0]
        ground_sensor_0 = [0, 0, 0, 0, 0]
        ground_sensor_1 = [0, 0, 0, 0, 0]

        while True:
            prox_horizontal = self.aseba.GetVariable("thymio-II", "prox.horizontal")
            prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.reflected")

            front_sensor_0.insert(0, prox_horizontal[0])
            del front_sensor_0[5]
            front_sensor_1.insert(0, prox_horizontal[1])
            del front_sensor_1[5]
            front_sensor_2.insert(0, prox_horizontal[2])
            del front_sensor_2[5]
            front_sensor_3.insert(0, prox_horizontal[3])
            del front_sensor_3[5]
            front_sensor_4.insert(0, prox_horizontal[4])
            del front_sensor_4[5]
            ground_sensor_0.insert(0, prox_ground[0])
            del ground_sensor_0[5]
            ground_sensor_1.insert(0, prox_ground[1])
            del ground_sensor_1[5]

            smoothed_front_0 = mean(front_sensor_0)
            smoothed_front_1 = mean(front_sensor_1)
            smoothed_front_2 = mean(front_sensor_2)
            smoothed_front_3 = mean(front_sensor_3)
            smoothed_front_4 = mean(front_sensor_4)
            smoothed_ground_0 = mean(ground_sensor_0)
            smoothed_ground_1 = mean(ground_sensor_1)

            if smoothed_ground_0 > 500 and smoothed_ground_1 > 500:
                print("White")
            else:
                print("Black")

    ############## Bus and aseba setup ######################################

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


# ------------------ Main loop here -------------------------

def main():
    robot = Thymio()

    # robot.sens()

    thread = Thread(target=robot.sens)
    thread.daemon = True
    thread.start()
    while True:
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
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")

## TEST
