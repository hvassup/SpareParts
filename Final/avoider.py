import random
from robot import Thymio

import os
import time

from cv2 import boundingRect
import cv2
from vision import detect_color, find_boundaries

from enums import Action
from q_learning import get_next_action, update_q_table, print_q_table

from camera import iso_test, look_for_color, saveImage, stop_camera, initCamera, takePicture, cam_size, take_continuous
from time import sleep
from threading import Thread

robot = None
running = True

in_safe_zone = False
comm_result = 0
is_silence = False

def do_threaded_task(task_fn):
    thread = Thread(target=task_fn)
    thread.daemon = True
    thread.start()

def listen():
    global comm_result, running
    print('Heyo i listening!')
    while running:
        comm_result = robot.receiveInformation()
        robot.sendInformation(2)
        if comm_result != 0:
            print('Received', comm_result)
            robot.restart_comms()
        sleep(0.1)
    robot.restart_comms()

boundaries = []

is_evil_in_sight = False
evil_boy_counter = 0
def look_for_color(image):
    global is_evil_in_sight, evil_boy_counter

    filtered_img = detect_color(image, 'red')

    # print('save image')
    
    boundaries = find_boundaries(filtered_img, 30)
    evil_boy_counter += 1
    picture_name = ''
    if len(boundaries) > 0:
        print('Detected evil boye')
        picture_name = f'{evil_boy_counter}_evil'
        is_evil_in_sight = True

    else:
        picture_name = f'{evil_boy_counter}_safe'
    
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
        return 1, -1
    elif center < (middle - margin):
        # Turn right
        return -1, 1
    return 0, 0

def turn_to_color():
    while True:
        print(boundaries)
        if len(boundaries) > 0:
            turn_direction = boundary_to_direction(boundaries[0])
            robot.drive(*turn_direction)
        else:
            robot.drive(0, 0)
        sleep(0.1)

def start_emitting_after_5_seconds():
    sleep(5)
    print('Emitting 2 again')
    robot.sendInformation(2)

#------------------ Main loop here -------------------------

def main():
    global robot, is_evil_in_sight

    robot = Thymio()
    # robot.turn_blue()
    robot.turn_blue()
    robot.restart_comms()
    robot.sendInformation(2)
    robot.disable_sensor_leds()
    
    sleep(1)
    
    # DO something
    do_threaded_task(listen)
    do_threaded_task(look)
    # robot.turn_degrees(180)
    # do_threaded_task(sense_bottom)
    print_q_table()
    # print(boundaries)
    # if len(boundaries) == 0:
    # saveImage()
    # sleep(10)
    # start_time = time.time()
    in_safe_zone = False
    # for _ in range(0, 10):
    #     takePicture()
    #     print('Took a pic!', time.time() - start_time)
    #     start_time = time.time()
    # print('Bye')
    # sleep(1000)

    while True:
        if robot.sens_front() > 1000:
            robot.drive(-1, 1)
            sleep(0.5)

        if in_safe_zone:
            res = comm_result
            if res == 2:
                print('Begin moving out of safe zone!')
                robot.turn_blue()
                robot.sendInformation(13)
                while robot.is_gray() == True:
                    robot.drive(1, 1)
                    sleep(0.1)
                print('Out of safe zone!')
                in_safe_zone = False
                do_threaded_task(start_emitting_after_5_seconds)
                robot.clear_gray()
                robot.drive(1, 1)
                sleep(1)
            continue
        
        if is_evil_in_sight:
            turn_dir = random.choice([90, -90])
            robot.turn_degrees(turn_dir)
            is_evil_in_sight = False

        if not in_safe_zone:
            res = comm_result
            if res == 1:
                robot.turn_purple()
                robot.stop()
                print('Im ded')
                ## Sad sound :(
                break
        
        if robot.is_gray():
            print('Found safe zone!')
            robot.stop()
            robot.turn_green()
            in_safe_zone = True
        else:
            current_state = robot.get_state()
            action_to_perform = Action(get_next_action(current_state))
            robot.perform_action(action_to_perform)
            next_state = robot.get_state()
            print('current_state:', current_state, 'action_to_perform:', action_to_perform, 'next_state:', next_state)
            update_q_table(current_state, action_to_perform, next_state)
    
    # for i in range(0, 101):
    #     robot.sense_bottom()
    #     sleep(0.1)
    
    # Set RX value to 0
    # print(robot.aseba.Events())
    # robot.aseba.SendEventName("prox.comm.rx",[0])

    # robot.becomeAvoider()
    # cnt = 0
    # start_time = time.time()
    # while True:
    #     res = robot.receiveInformation()
    #     if res != 0:
    #         # print(res)
    #         cnt += 1
    #         if cnt % 20 == 0:
    #             print(time.time() - start_time)
    #             start_time = time.time()
    #     robot.restart_comms()
    #     sleep(0.1)
    # initCamera()
    # findtag()
    

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
