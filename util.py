import numpy as np
import cv2 as cv
import os

# The function gets the colour ranges for inputed colours
# The function also deals with the colour such as red that 'wrap' around the HSV colour wheel
def get_limits(hsv_colour, error=20) :

    # Setting initial parameters for hsv
    h = hsv_colour[0]
    s = hsv_colour[1]
    v = hsv_colour[2]
    s_lower = 100
    s_upper = 255
    v_lower = 100
    v_upper = 255

    # The case for which the colour is black
    if h == 0:
        lowerLimit = 0, 0, 0
        upperLimit = 180, 60, 80

        lower = np.array(lowerLimit, dtype=np.uint8)
        upper = np.array(upperLimit, dtype=np.uint8)

        return [(lower, upper)]

    # The case for which the colour is red/orange
    if h < 10:
        lowerLimit1 = 0, s_lower, v_lower
        upperLimit1 = h + 10, s_upper, v_upper
        lowerLimit2 = 179 - (10 - h), s_lower, v_lower
        upperLimit2 = 179, s_upper, v_upper

        lower1 = np.array(lowerLimit1, dtype=np.uint8)
        upper1 = np.array(upperLimit1, dtype=np.uint8)
        lower2 = np.array(lowerLimit2, dtype=np.uint8)
        upper2 = np.array(upperLimit2, dtype=np.uint8)

        return [(lower1, upper1), (lower2, upper2)]

    # The case for which the colour is pink/purple
    if (h > 169):
        lowerLimit1 = h - 10, s_lower, v_lower
        upperLimit1 = 179, s_upper, v_upper
        lowerLimit2 = 0, s_lower, v_lower
        upperLimit2 =  (h + 10) - 179, s_upper, v_upper

        lower1 = np.array(lowerLimit1, dtype=np.uint8)
        upper1 = np.array(upperLimit1, dtype=np.uint8)
        lower2 = np.array(lowerLimit2, dtype=np.uint8)
        upper2 = np.array(upperLimit2, dtype=np.uint8)

        return [(lower1, upper1), (lower2, upper2)]

    # Handles every other colour in HSL colour wheel
    if (h == 30):
        lowerLimit1 = 15, 40, 100
        upperLimit1 = 35, 255, 255
        lower1 = np.array(lowerLimit1, dtype=np.uint8)
        upper1 = np.array(upperLimit1, dtype=np.uint8)
        return [(lower1, upper1)]
    else:
        lowerLimit = max(h - error, 0), s_lower, v_lower
        upperLimit = min(h + error, 179), s_upper, v_upper

        lowerLimit = np.array(lowerLimit, dtype=np.uint8)
        upperLimit = np.array(upperLimit, dtype=np.uint8)
        return [(lowerLimit, upperLimit)]

# This function gets the largest contours extracted from camera view
def get_largest_contour(contours):
    if contours:
        return max(contours, key=cv.contourArea)
    else :
      return None