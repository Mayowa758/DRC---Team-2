import cv2 as cv
import numpy as np
from colour_detect import *
from road_detect import *

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
    # center of the camera frame
    frame_center = frame.shape[1] // 2

    # left
    if arrow_Mx < frame_center - 20:
        error = error - correction_factor
        print("left")
    # right
    elif arrow_Mx > frame_center + 20:
        error = error + correction_factor
        print("right")
    # if unsure center
    else:
        error = error
        print("center")
    return error

# Function that deals will obstacles
def obstacle_detection():
    return 0;

# Function that can deal with the finish line
def finish_line():
    return 0;
