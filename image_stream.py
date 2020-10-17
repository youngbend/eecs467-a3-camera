import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import cv2
import lcm
from lcmtypes.image_t import image_t

camera_resolution = (640,480)
camera_framerate = 16

# Change this
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

camera = PiCamera()
camera.resolution = camera_resolution
camera.framerate = camera_framerate
rawCapture = PiRGBArray(camera, size=camera_resolution)

for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    image = frame.array

    # Comment this out if camera is mounted on top
    image = cv2.flip(image, -1)

    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    image_packet = image_t()
    image_packet.utime = int(time.time() * 1e9)

    image_packet.height = image.shape[0]
    image_packet.width = image.shape[1]
    image_packet.pixelformat = cv2.CV_8UC3
    image_packet.size = image.shape[0] * image.shape[1]
    image_packet.data = image.tobytes()

    lc.publish("MBOT_IMAGE_STREAM", image_packet.encode())

    rawCapture.truncate(0)

