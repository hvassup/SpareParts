# imports the camera

from picamera import PiCamera
import time
import numpy as np
import cv2
import apriltag

#initialize

camera = PiCamera()

def takePicture():
    #print("Taking picture!")
    camera.start_preview()
    #time.sleep(5)
    #we capture to openCV compatible format
    #you might want to increase resolution
    camera.resolution = (640, 480)
    camera.framerate = 24
    #time.sleep(2)
    
    image = np.empty((480, 640, 3), dtype=np.uint8)
    camera.capture(image, 'bgr')
    flippedImage = cv2.flip(image, -1)
    cv2.imwrite('example.png', flippedImage)
    camera.stop_preview()
    #print("saved image to example.png")

def checkTag():
    img = cv2.imread('example.png',0)
    #print("Detecting AprilTags...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(img)
    #print("{} total AprilTags detected".format(len(results)))
    if results == []:
        return(0)
    else:
        return(results[0][1])

while True:
    takePicture()
    tag = checkTag()
    if tag != 0:
        print(tag)
        break
