import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import pygame
import time
import sys
import cv2
import lcm
from lcmtypes.simple_motor_command_t import simple_motor_command_t
import fisheye_correction

camera_resolution = [640,480]
camera_framerate = 16

FORWARD_VEL_CONST = 0.3
ANGULAR_VEL_CONST = 2

fourcc = cv2.VideoWriter_fourcc(*'MP42')
video = cv2.VideoWriter('demo.avi', fourcc, camera_framerate, tuple(camera_resolution))

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
pygame.init()
pygame.display.set_caption("Camera Feed")
screen = pygame.display.set_mode(camera_resolution)
camera = PiCamera()
camera.resolution = tuple(camera_resolution)
camera.framerate = camera_framerate
rawCapture = PiRGBArray(camera, size=tuple(camera_resolution))

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array

    # Comment this out if your camera is mounted on the top
    image = cv2.flip(image, -1)

    # Correct fisheye distortion (should NOT be corrected if using with ORBSLAM3)
    image = fisheye_correction.undistort(image)

    video.write(image)

    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    screen.fill([0,0,0])
    image = image.swapaxes(0,1)
    image = pygame.surfarray.make_surface(image)
    screen.blit(image, (0,0))
    pygame.display.update()

    command = simple_motor_command_t()
    command.utime = int(time.time() * 1000000)
    command.angular_velocity = 0
    command.forward_velocity = 0

    done = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            cv2.destroyAllWindows()
            done = True
            break

    if done:
        break

    key_input = pygame.key.get_pressed()
    if key_input[pygame.K_LEFT]:
        command.angular_velocity += ANGULAR_VEL_CONST
    if key_input[pygame.K_UP]:
        command.forward_velocity += FORWARD_VEL_CONST
    if key_input[pygame.K_RIGHT]:
        command.angular_velocity -= ANGULAR_VEL_CONST
    if key_input[pygame.K_DOWN]:
        command.forward_velocity -= FORWARD_VEL_CONST
    if key_input[pygame.K_q]:
        pygame.quit()
        cv2.destroyAllWindows()
        break

    lc.publish("MBOT_MOTOR_COMMAND_SIMPLE", command.encode())

    rawCapture.truncate(0)

video.release()

