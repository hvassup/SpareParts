# imports the camera

from picamera import PiCamera
import time
import numpy as np
import cv2

#initialize

camera = PiCamera()

def testCamera():
    print("Camera test")
    camera.start_preview()
    time.sleep(5)
    #we capture to openCV compatible format
    #you might want to increase resolution
    camera.resolution = (320, 240)
    camera.framerate = 24
    time.sleep(2)
    image = np.empty((240, 320, 3), dtype=np.uint8)
    camera.capture(image, 'bgr')
    cv2.imwrite('out.png', image)
    camera.stop_preview()
    print("saved image to out.png")

testCamera()