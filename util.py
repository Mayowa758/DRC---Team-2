import numpy as np
import cv2 as cv

# The function also deals with if red as that colour 'wraps' around the wheel
def get_limits(color) :

    c = np.uint8([[color]])
    hsvC = cv.cvtColor(c, cv.COLOR_BGR2HSV)

    h = hsvC[0][0][0]

    if h < 10:
        lowerLimit1 = 0, 100, 100
        upperLimit1 = h + 10, 255, 255
        lowerLimit2 = 180 + (h - 10), 100, 100
        upperLimit2 = 180, 255, 255

        lower1 = np.array(lowerLimit1, dtype=np.uint8)
        upper1 = np.array(upperLimit1, dtype=np.uint8)
        lower2 = np.array(lowerLimit2, dtype=np.uint8)
        upper2 = np.array(upperLimit2, dtype=np.uint8)
        
        return [(lower1, upper1), (lower2, upper2)]
    
    if (h > 170):
        lowerLimit1 = h - 10, 100, 100
        upperLimit1 = 180, 255, 255
        lowerLimit2 = 0, 100, 100
        upperLimit2 =  (h + 10) - 180, 255, 255

        lower1 = np.array(lowerLimit1, dtype=np.uint8)
        upper1 = np.array(upperLimit1, dtype=np.uint8)
        lower2 = np.array(lowerLimit2, dtype=np.uint8)
        upper2 = np.array(upperLimit2, dtype=np.uint8)

        return [(lower1, upper1), (lower2, upper2)]
    else:

        lowerLimit = h - 10, 100, 100
        upperLimit = h + 10, 255, 255

        lowerLimit = np.array(lowerLimit, dtype=np.uint8)
        upperLimit = np.array(upperLimit, dtype=np.uint8)
        return [(lowerLimit, upperLimit)]