import cv2 as cv
import numpy as np
from colour_detect import *
from road_detect import get_largest_contour

# Function that deals with the turning challenge
def arrow_detection(frame, error):
    correction_factor = 100;
    black_range = get_limits(black)
    arrow_mask = get_mask(frame, black_range, kernel)

    # find contours of black arrow
    arrow_contours, _ = cv.findContours(arrow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not arrow_contours:
        return error

    arrow_area = get_largest_contour(arrow_contours)
    arrow_M = cv.moments(arrow_area)

    # centroid of the black arrow
    arrow_Mx = int(arrow_M['m10']/arrow_M['m00'])
    if arrow_M['m00']:
        return error
    # center of the camera frame
    frame_center = frame.shape[1] // 2

    # left
    if arrow_Mx < frame_center - 20:
        error -= correction_factor
        print("left")
    # right
    elif arrow_Mx > frame_center + 20:
        error += correction_factor
        print("right")
    # if unsure center
    else:
        print("center")
    return error

# Function that deals will obstacles
def obstacle_detection(hsv_img, error):

    correction_factor = 25
    purple_range = get_limits(purple)
    purple_mask = get_mask(hsv_img, purple_range, kernel)
    get_object, _ = cv.findContours(purple_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not get_object:
        return error

    object_area = get_largest_contour(get_object)
    object_M = cv.moments(object_area)
    object_Mx = int(object_M["m10"]/object_M["m00"])
    if object_M['m00']:
        return error

    frame_center = hsv_img.shape[1] // 2
    if object_Mx > frame_center:
        error += correction_factor
    else:
        error -= correction_factor
    return error

