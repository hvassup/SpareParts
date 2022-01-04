from time import sleep
import time
import io
from vision import detect_color, find_boundaries
from picamera import PiCamera
import picamera.array
import cv2
import numpy as np
# ----------------------- Camera init ------------------------
camera = None
cam_size = (640, 480)
# cam_size = (320, 240)
def initCamera():
    global camera
    # initialize camera & time intervals for image capture
    print('Camera starting up')
    camera = PiCamera()

    camera.resolution = cam_size
    camera.framerate = 24

def takePicture():
    image = np.empty((cam_size[1], cam_size[0], 3), dtype=np.uint8)
    camera.capture(image, 'bgr')
    flippedImage = cv2.flip(image, -1)
    return flippedImage

def iso_test(iso, exposure, callback):
    with picamera.PiCamera() as camera:
            camera.resolution = cam_size
            # camera.image_effect = 'saturation'
            camera.framerate = 60
            image = np.empty((cam_size[1], cam_size[0], 3), dtype=np.uint8)
            camera.capture(image, 'bgr')
            flippedImage = cv2.flip(image, -1)
            callback(flippedImage)


# Loop which takes images with a delay
def saveImage():
    print('Saving image')
    cv2.imwrite(f"test.jpg", takePicture())

def take_continuous(callback):
    with picamera.PiCamera() as camera:
        camera.resolution = cam_size
        camera.image_effect = 'saturation'
        camera.iso = 100
        camera.framerate = 60
        stream = io.BytesIO()
        # , use_video_port=True
        for _ in camera.capture_continuous(stream, format='jpeg'):
            image_stream = np.asarray(bytearray(stream.getvalue()), dtype=np.uint8)
            
            image = cv2.imdecode(image_stream, cv2.IMREAD_COLOR)
            
            flippedImage = cv2.flip(image, -1)
            # start_time = time.time()
            callback(flippedImage)
            # print('Processed picture in', time.time() - start_time)
            
            stream.seek(0)
            stream.truncate(0)


def look_for_color(color):
    print('Looking for', color)
    # start_time = time.time()
    img = takePicture()
    # print('Took picture in', time.time() - start_time)
    # start_time = time.time()
    filtered_img = detect_color(img, color)
    
    print('save')
    cv2.imwrite('taeest.jpg', filtered_img)
    print(np.sum(filtered_img))
    boundaries = find_boundaries(filtered_img, 30)
    # print('Found boundaries in', time.time() - start_time)
    print(boundaries)
    return boundaries

def stop_camera():
    camera.close()

# --------------------- init script end ----------------------