import cv2 as cv
import numpy as np
from colour_detect import *
from road_detect import get_largest_contour

# Function that deals with the turning challenge
def arrow_detection(frame, error):
    # Some value for correcting PID
    correction_factor = 100;

    # find contours of black arrow
    black_range = get_limits(black)
    arrow_mask = get_mask(frame, black_range, kernel)
    arrow_contours, _ = cv.findContours(arrow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not arrow_contours:
        return error
    arrow_area = get_largest_contour(arrow_contours)
    arrow_M = cv.moments(arrow_area)

    # calculate the centroid of the black arrow
    arrow_Mx = int(arrow_M['m10']/arrow_M['m00'])
    if arrow_M['m00'] == 0:
        return error

    # center of the camera frame
    frame_center = frame.shape[1] // 2

    # Perform the correction
    if arrow_Mx > frame_center:
        error -= correction_factor
        print("left")
    else:
        error += correction_factor
        print("right");
    return error

# Function that deals with obstacles
def obstacle_detection(hsv_img, error, img):
    # Some value for correcting the PID
    correction_factor = 25
    # Getting the contour of obstacle
    purple_range = get_limits(purple)
    purple_mask = get_mask(hsv_img, purple_range, kernel)
    object_contour, _ = cv.findContours(purple_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not object_contour:
        return error

    # Getting the center of mass of the object
    object_area = get_largest_contour(object_contour)
    object_M = cv.moments(object_area)
    object_Mx = int(object_M["m10"]/object_M["m00"])
    if object_M['m00'] == 0:
        return error

    # Center of the camera frame
    frame_center = hsv_img.shape[1] // 2

    # Perform the correction
    if object_Mx > frame_center:
        error -= correction_factor
        print("left")
    else:
        error += correction_factor
        print("right")
    return error

