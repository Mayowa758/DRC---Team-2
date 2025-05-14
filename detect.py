import cv2 as cv
import numpy as np

from util import get_limits

def get_mask(img, lower, upper):
    return cv.inRange(img, lower, upper)

def mask(blue, yellow, green, red, purple):
    return blue + yellow + green + red + purple

video = cv.VideoCapture(0);

blue = [255, 0, 0]
yellow = [255, 255, 0]
red = [0, 0, 255]
purple = [128, 0, 128]
green = [0, 255, 0]

while True:

    _, img = video.read()
    img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # BLUE and YELLOW are for road lines
    # RED and PURPLE obstacle detection draw rectangles maybe
    # GREEN is the end

    # This is for BLUE
    blue_lower, blue_upper = get_limits(color=blue)

    # This is for YELLOW
    yellow_lower, yellow_upper = get_limits(color=yellow)

    # This is for RED
    red_lower, red_upper = get_limits(color=red)

    # This is for PURPLE
    purple_lower, purple_upper = get_limits(color=purple)

    # This is for GREEN
    green_lower, green_upper = get_limits(color=green)

    mask_blue = get_mask(img, blue_lower, blue_upper)
    mask_yellow = get_mask(img, yellow_lower, yellow_upper)

    mask_red = get_mask(img, red_lower, red_upper)
    mask_purple = get_mask(img, purple_lower, purple_upper)

    mask_green = get_mask(img, green_lower, green_upper)

    master_mask = mask(mask_blue, mask_yellow, mask_red, mask_purple, mask_green)

    # to detect one of the colours 
    res = cv.bitwise_and(img, img, mask =master_mask)


    # contours, hieracrhy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # cv.imshow("mask", mask)
    cv.imshow("webcam", res)

    # if len(contours) != 0 :
    #     for contour in contours:
    #         if cv.contourArea(contour) > 500:
    #             x, y, w, h = cv.boundingRect(contour)
    #             cv.rectangle(img, (x,y), (x + w + y + h), (0, 0, 255), 3)

    
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv.destroyAllWindows()