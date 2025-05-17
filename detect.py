import cv2 as cv
from PIL import Image
import numpy as np

from util import get_limits

def get_mask(img, range):
    mask = None
    for lower, upper in range:
        partial_mask = cv.inRange(img, lower, upper)
        if mask is None:
            mask = partial_mask
        else:
            mask = cv.bitwise_or(mask, partial_mask)


    return cv.inRange(img, lower, upper)

def colour_detect_test(mask_):
    mask_ = Image.fromarray(mask_blue)

    bbox = mask_.getbbox()
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        img = cv.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 5)
    return img

def mask(blue, yellow, green, red, purple):
    return blue + yellow + green + red + purple

video = cv.VideoCapture(0);

blue = [255, 0, 0]
yellow = [0, 255, 255]
red = [0, 0, 255]
purple = [128, 0, 128]
green = [0, 255, 0]

while True:

    _, img = video.read()
    img = cv.GaussianBlur(img, (13, 13), 0)
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # BLUE and YELLOW are for road lines
    # RED and PURPLE obstacle detection draw rectangles maybe
    # GREEN is the end

    blue_range = get_limits(blue)
    yellow_range = get_limits(yellow)
    red_range = get_limits(red)
    purple_range = get_limits(purple)
    green_range = get_limits(green)

    mask_blue = get_mask(hsv_img, blue_range)
    mask_yellow = get_mask(hsv_img, yellow_range)
    

        # lower mask (0-10)
    mask_red = get_mask(hsv_img, red_range)
    mask_purple = get_mask(hsv_img, purple_range)
    mask_green = get_mask(hsv_img, green_range)

    master_mask = mask(mask_blue, mask_yellow, mask_red, mask_purple, mask_green)

    res = cv.bitwise_and(img, img, mask =master_mask)
    # contours, hieracrhy = cv.findContours(mask_yellow, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # print(bbox)

    # if len(contours) != 0:
    #     for contour in contours:
    #         if cv.contourArea(contour) > 500:
    #             x, y, w, h = cv.boundingRect(contour)
    #             cv.rectangle(img, (x,y), (x + w, y + h), (0, 0, 255), 3)

    cv.imshow("mask", master_mask)
    cv.imshow("webcam", img)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv.destroyAllWindows()