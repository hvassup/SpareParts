#!/usr/bin/python3
import os
# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")
import matplotlib.pyplot as plt
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

    def sens(self):
        prox_horizontal = self.aseba.GetVariable("thymio-II", "prox.horizontal")
        print("Sensing:")
        print(prox_horizontal[0])
        print(prox_horizontal[1])
        print(prox_horizontal[2])
        print(prox_horizontal[3])
        print(prox_horizontal[4])
    
    def sense_bottom(self):
        prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.ambiant")
        print(f'Ambiant,{prox_ground[0]},{prox_ground[1]}')
        
        prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.reflected")
        print(f'Reflected,{prox_ground[0]},{prox_ground[1]}')
        
        prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.delta")
        print(f'Delta,{prox_ground[0]},{prox_ground[1]}')

    
    def set_color(self, color):
        self.aseba.SendEventName("leds.top", color)
        self.aseba.SendEventName("leds.bottom.right", color)
        self.aseba.SendEventName("leds.bottom.left", color)
    
    def turn_red(self):
        self.set_color([255, 0, 0])

    def turn_orange(self):
        self.set_color([255, 4, 0])

    def turn_blue(self):
        self.set_color([0, 0, 255])

    def turn_green(self):
        self.set_color([0, 255, 0])

    def turn_purple(self):
        self.set_color([255, 0, 255])

    def sendInformation(self, number):
        self.aseba.SendEventName("prox.comm.tx", [number])

    def receiveInformation(self):
        rx = self.aseba.GetVariable("thymio-II", "prox.comm.rx")
        print(rx[0])


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
        # Set RX value to 0
        # asebaNetwork.SendEventName("prox.comm.rx",[0])

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



#------------------ Main loop here -------------------------

def main():
    robot = Thymio()

    #robot.sens() 

    #thread = Thread(target=robot.sens)
    #thread.daemon = True
    #thread.start()

#    thread = Thread(target=robot.led_control())
 #   thread.daemon = True
  #  thread.start()

    # robot.drive(200, 200)
    # sleep(5)
    # robot.stop()
    # robot.turn_red()
    # sleep(1)
    # robot.turn_orange()
    # sleep(1)
    # robot.turn_blue()
    # sleep(1)
    # robot.turn_green()
    # sleep(1)
    # robot.turn_purple()
    # sleep(1)
    # for i in range(0, 101):
    #     robot.sense_bottom()
    #     sleep(0.1)

    robot.receiveInformation()
    robot.sendInformation(1)
    robot.receiveInformation()
    # while True:
    #     robot.sense_bottom()
    #     sleep(1)
       

#------------------- Main loop end ------------------------

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Stopping robot")
        exit_now = True
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")
