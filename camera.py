# imports the camera

from picamera import PiCamera
import time
import numpy as np
import cv2
import apriltag

#initialize

camera = PiCamera()

def takePicture():
    print("Taking picture!")
    camera.start_preview()
    time.sleep(5)
    #we capture to openCV compatible format
    camera.resolution = (640, 480)
    camera.framerate = 24
    time.sleep(2)

    image = np.empty((480, 640, 3), dtype=np.uint8)
    camera.capture(image, 'bgr')
    flippedImage = cv2.flip(image, -1)
    cv2.imwrite('example.png', flippedImage)
    camera.stop_preview()
    print("saved image to example.png")

def checkTag():
    img = cv2.imread('example.png',0)
    print("[INFO] detecting AprilTags...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(img)
    print("[INFO] {} total AprilTags detected".format(len(results)))
    print("Tag: ", results)


takePicture()
checkTag()
