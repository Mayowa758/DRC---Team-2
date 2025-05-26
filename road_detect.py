import cv2 as cv
import numpy as np
from util import get_limits
from configure.undistort_data import *

# from colour_detect import get_mask

# from colour_detect import mask_blue, mask_yellow
video = cv.VideoCapture(0)

# Colour in HSV
blue = [110, 255, 255]
yellow = [30, 255, 255]

def road_mask(blue, yellow):
    new_mask = blue | yellow
    return new_mask

def get_mask(img, range, kernel):
    mask = None
    for lower, upper in range:
        partial_mask = cv.inRange(img, lower, upper)
        if mask is None:
            mask = partial_mask
        else:
            mask = cv.bitwise_or(mask, partial_mask)
    mask = cv.dilate(mask, kernel)
    return mask


# Change the frame rate of the camera
video.set(cv.CAP_PROP_FRAME_WIDTH, 640)
video.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
video.set(cv.CAP_PROP_FPS, 30)

h, w = video.shape[:2]
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv.CV_16SC2)

while True:
    __, img = video.read()

    img = cv.GaussianBlur(img, (13, 13), 0)
    img = cv.remap(img, mapx, mapy, interpolation=cv.INTER_LINEAR)
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    kernel = np.ones((5,5), "uint8")

    blue_range = get_limits(blue)
    yellow_range = get_limits(yellow)
    blue_mask = get_mask(hsv_img, blue_range, kernel)
    yellow_mask = get_mask(hsv_img, yellow_range, kernel)
    drive_mask = road_mask(blue_mask, yellow_mask)

    cv.imshow('drive_mask', drive_mask)
    cv.imshow('actual', img)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv.destroyAllWindows()