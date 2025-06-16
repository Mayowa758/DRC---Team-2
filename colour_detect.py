import cv2 as cv
from PIL import Image
import numpy as np
from util import get_limits
from configure.undistort_data import *

# Creates the colour mask obtained from colour range specified
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

# Below 2 different functions to draw boxes around the specified colours red, green etc.

# PIL (Python Imaging Library)
def colour_detect_test(img, mask, label, colour):
    mask_ = Image.fromarray(mask)
    colour = hsv_to_bgr(colour)
    bbox = mask_.getbbox()
    if bbox is not None:
        x, y, w, h = bbox
        img = cv.rectangle(img, (x, y), (w, h), colour, 5)
        cv.putText(img, label, (x, y), cv.FONT_HERSHEY_COMPLEX_SMALL, 1.0, colour)

# BOUNDARY BOX (rectangles)
def draw_boundary_boxes(img, mask, label, colour):
    colour = hsv_to_bgr(colour)
    for i in range(len(mask)):
        contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            for contour in contours:
                area = cv.contourArea(contour)
                if(area > 500):
                    x, y, w, h = cv.boundingRect(contour)
                    cv.rectangle(img, (x,y), (x + w, y + h), colour, 2)
                    cv.putText(img, label, (x,y), cv.FONT_HERSHEY_COMPLEX_SMALL, 1.0, colour)

# This function combines all the different masks together
def mask(blue, yellow, green, red, purple):
    # bitwise or operation
    combined  = blue | yellow | green | red | purple
    return combined

# Function to convert hsv colour values to bgr
def hsv_to_bgr(colour):
    hsv_np = np.uint8([[colour]])
    bgr_np = cv.cvtColor(hsv_np, cv.COLOR_HSV2BGR)
    return tuple(int(x) for x in bgr_np[0][0])

# HSV values for the colours we will use
blue = [110, 160, 180]
yellow = [22, 52, 230]
red = [0, 255, 255]
purple = [174, 81, 231]
green = [60, 255, 255]
black = [0, 0, 0]
kernel = np.ones((5,5), "uint8")

def run_video():
    video = cv.VideoCapture(0)

    video.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    video.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    video.set(cv.CAP_PROP_FPS, 30)

    # Might need this idk
    BLUE = 0
    YELLOW = 1
    RED = 2
    PURPLE = 3
    GREEN = 4

    ret, frame = video.read()
    if not ret or frame is None:
        raise RuntimeError("Failed to read frame from camera")

    h, w = frame.shape[:2]
    mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv.CV_16SC2)

    while True:
        _, img = video.read()
        img = cv.remap(img, mapx, mapy, interpolation=cv.INTER_LINEAR)
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

        mask_blue = get_mask(hsv_img, blue_range, kernel)
        mask_yellow = get_mask(hsv_img, yellow_range, kernel)
        mask_red = get_mask(hsv_img, red_range, kernel)
        mask_purple = get_mask(hsv_img, purple_range, kernel)
        mask_green = get_mask(hsv_img, green_range, kernel)

        colour_masks = [
            {"mask": mask_blue, "label": "blue", "colour": blue},
            {"mask": mask_yellow, "label": "yellow", "colour": yellow},
            {"mask": mask_red, "label": "red", "colour": red},
            {"mask": mask_purple, "label": "purple", "colour": purple},
            {"mask": mask_green, "label": "green", "colour": green},
        ]

        # Uses PIL
        for entry in colour_masks:
            colour_detect_test(img, entry["mask"], entry["label"], entry["colour"])

        # Uses contours
        # for entry in colour_masks:
        #     draw_boundary_boxes(img, entry["mask"], entry["label"], entry["colour"])

        master_mask = mask(mask_blue, mask_yellow, mask_red, mask_purple, mask_green)
        # res = cv.bitwise_and(img, img, mask =master_mask)
        # contours, hierarchy = cv.findContours(mask_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        cv.imshow("mask", master_mask)
        cv.imshow("webcam", img)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    video.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    run_video()