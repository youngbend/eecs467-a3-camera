# Function for undistorting images from the MBOT
import cv2
import numpy as np

# Replace with your camera coefs
DIM=(640, 480)
K=np.array([[182.32226761843935, 0.0, 320.97455065805013], [0.0, 182.34331322907786, 248.6347013989196], [0.0, 0.0, 1.0]])
D=np.array([[0.06795469584121445], [-0.0442543677291955], [0.013957869619636992], [-0.0022498280235961608]])

map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def undistort(img):
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img


