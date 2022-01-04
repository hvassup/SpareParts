import random
from robot import Thymio

import os
import time

from cv2 import boundingRect
import cv2
from vision import detect_color, find_boundaries

from enums import Action, State
from q_learning import get_next_action, update_q_table, print_q_table

from camera import iso_test, look_for_color, saveImage, stop_camera, initCamera, takePicture, cam_size, take_continuous
from time import sleep
from threading import Thread

robot = None
running = True

in_safe_zone = False
comm_result = 0
is_silence = False
boundary_timestamp = 0

def do_threaded_task(task_fn):
    thread = Thread(target=task_fn)
    thread.daemon = True
    thread.start()

def speak():
    print('Heyo i am sending!')
    while running:
        robot.sendInformation(1)

boundaries = []

is_evil_in_sight = False
evil_boy_counter = 0
def look_for_color(image):
    global is_evil_in_sight, evil_boy_counter, boundaries

    filtered_img = detect_color(image, 'blue')
    
    boundaries = find_boundaries(filtered_img, 30)
    if len(boundaries) > 0:
        print('Saw blue!')
    
    # evil_boy_counter += 1
    # picture_name = ''
    # if len(boundaries) > 0:
    #     print('Detected evil boye')
    #     picture_name = f'{evil_boy_counter}_evil'
    #     is_evil_in_sight = True

    # else:
    #     picture_name = f'{evil_boy_counter}_safe'
    
    # cv2.imwrite(f'{picture_name}.jpg', image)
    # cv2.imwrite(f'{picture_name}_filtered.jpg', filtered_img)

def look():
    take_continuous(look_for_color)

def sense_bottom():
    robot.sense_bottom()

def boundary_to_direction(boundary):
    center = (boundary[0] + boundary[1]) / 2
    middle = cam_size[0] / 2
    margin = 50
    if center > (middle + margin):
        # Turn right
        return 1, 0.35
    elif center < (middle - margin):
        # Turn right
        return 0.35, 1
    else:
        return 1, 1

def turn_to_color(boundary):
    global boundary_timestamp
    print('Moving towards:', boundary)
    turn_direction = boundary_to_direction(boundary)
    robot.drive(*turn_direction)
    
    boundary_timestamp = time.time()
    

#------------------ Main loop here -------------------------

def main():
    global robot, is_evil_in_sight, boundaries

    robot = Thymio(True)
    robot.turn_red()

    robot.restart_comms()
    robot.sendInformation(1)
    robot.disable_sensor_leds()
    
    sleep(1)
    
    # DO something
    do_threaded_task(speak)
    do_threaded_task(look)
    # robot.turn_degrees(180)
    # do_threaded_task(sense_bottom)
    print_q_table()

    while True:
        if robot.is_gray():
            robot.turn_yellow()
        else:
            robot.turn_red()

        if robot.sens_front() > 1000:
            robot.drive(-1, 1)
            sleep(0.5)
        if len(boundaries) > 0 and not robot.sensor_state == State.BothSensors:
            turn_to_color(boundaries[0])
            sleep(0.05)
            # boundaries = []
            
        else:
            if time.time() - boundary_timestamp > 5:
                robot.drive(-0.2, 0.2)
            else:
                current_state = robot.get_state()
                action_to_perform = Action(get_next_action(current_state))
                robot.perform_action(action_to_perform)
                next_state = robot.get_state()
                print('current_state:', current_state, 'action_to_perform:', action_to_perform, 'next_state:', next_state)
                update_q_table(current_state, action_to_perform, next_state)
    

#------------------- Main loop end ------------------------

def tear_down():
    print("Stopping robot")
    if robot != None:
        robot.stop()
    sleep(1)
    os.system("pkill -n asebamedulla")
    print("asebamodulla killed")

if __name__ == '__main__':
    try:
        main()
        exit_now = True
        tear_down()
    except KeyboardInterrupt:
        print('Keyboard interrupt')
        exit_now = True
        tear_down()
    except Exception as e:
        print('Frick!')
        tear_down()
        raise e
