import numpy as np
import cv2 as cv

# The function also deals with if red as that colour 'wraps' around the wheel
def get_limits(hsv_colour, error=10) :

    h = hsv_colour[0]
    s = hsv_colour[1]
    v = hsv_colour[2]
    s_lower = 100
    s_upper = 255
    v_lower = 100
    v_upper = 255
    if h == 0:
        lowerLimit = 0, 0, 0
        upperLimit = 0, 0, 0
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
    else:

        lowerLimit = max(h - error, 0), s_lower, v_lower
        upperLimit = min(h + error, 179), s_upper, v_upper

        lowerLimit = np.array(lowerLimit, dtype=np.uint8)
        upperLimit = np.array(upperLimit, dtype=np.uint8)
        return [(lowerLimit, upperLimit)]