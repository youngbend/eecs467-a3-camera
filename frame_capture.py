import numpy as np
import math
import time
from picamera import PiCamera
from picamera.array import PiRGBArray
import matplotlib.pyplot as plt
import sys
import os
import cv2
import fisheye_correction

record_time = 20
camera_resolution = [640,480]
camera_framerate = 8

if not os.path.exists('captures'):
    os.makedirs('captures')

camera = PiCamera()
camera.resolution = tuple(camera_resolution)
camera.framerate = camera_framerate
rawCapture = PiRGBArray(camera, size=tuple(camera_resolution))
time.sleep(0.5)

frame_counter = 0
start_time = time.time()

for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    image = frame.array

    # Comment this out if your camera is mounted on the top
    image = cv2.flip(image, -1)

    image = fisheye_correction.undistort(image)

    plt.imsave("captures/img" + str(frame_counter) + ".jpg", image)

    frame_counter += 1
    rawCapture.truncate(0)

    if time.time() - start_time > record_time:
        break


