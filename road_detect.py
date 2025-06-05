import cv2 as cv
import numpy as np
from util import get_limits
from configure.undistort_data import *
from colour_detect import *

# from colour_detect import mask_blue, mask_yellow
video = cv.VideoCapture(0)
window_width = 640
window_height = 480

def road_mask(blue, yellow):
    new_mask = blue | yellow
    return new_mask

def perspective_transform(img):
    # These values may need to change
    tl = (150, 300)
    bl = (0, 472)
    tr = (480,  300)
    br = (600, 472)

    src_points = np.float32([tl, bl, tr, br])
    dst_points = np.float32([[0,0], [0,480], [640, 0], [640, 480]])

    cv.circle(img, tl, 5, (0, 0, 255), -1)
    cv.circle(img, bl, 5, (0, 0, 255), -1)
    cv.circle(img, tr, 5, (0, 0, 255), -1)
    cv.circle(img, br, 5, (0, 0, 255), -1)

    matrix = cv.getPerspectiveTransform(src_points, dst_points)
    transformed_frame = cv.warpPerspective(img, matrix, (window_width, window_height))

    return transformed_frame

def get_largest_contour(contours):
    return cv.contourArea(contours, cv.contourArea)

# Change the frame rate of the camera
video.set(cv.CAP_PROP_FRAME_WIDTH, window_width)
video.set(cv.CAP_PROP_FRAME_HEIGHT, window_height)
video.set(cv.CAP_PROP_FPS, 30)

ret, frame = video.read()
if not ret or frame is None:
    raise RuntimeError("Failed to read frame from camera")
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (window_width, window_height), 
                                                 0, (window_width, window_height))
h, w = frame.shape[:2]
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv.CV_16SC2)

while True:
    _, img = video.read()
    prev = img
    img = cv.remap(img, mapx, mapy, interpolation=cv.INTER_LINEAR)
    img = cv.GaussianBlur(img, (13, 13), 0)
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    blue_range = get_limits(blue)
    yellow_range = get_limits(yellow)

    blue_mask = get_mask(hsv_img, blue_range, kernel)
    yellow_mask = get_mask(hsv_img, yellow_range, kernel)
    drive_mask = road_mask(blue_mask, yellow_mask)

    # Bird's eye transformation
    transformed_frame = perspective_transform(img)
    hsv_img_bv = cv.cvtColor(transformed_frame, cv.COLOR_BGR2HSV)
    blue_mask_bv = get_mask(hsv_img_bv, blue_range, kernel)
    yellow_mask_bv = get_mask(hsv_img_bv, yellow_range, kernel)
    drive_mask_bv = road_mask(blue_mask_bv, yellow_mask_bv)

    cv.imshow('drive_mask_bv', drive_mask_bv)
    # cv.imshow('blue', blue_mask)
    # cv.imshow('yellow', yellow_mask)

    blue_contour, _ = cv.findContours(blue_mask_bv, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    yellow_contour, _ = cv.findContours(blue_mask_bv, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if blue_contour and yellow_contour:

        blue_line = get_largest_contour(blue_contour)
        yellow_line = get_largest_contour(yellow_contour)

        M_blue = cv.moments(blue_line)
        M_yellow = cv.moments(yellow_line)

        if M_blue["00"] != 0 and M_yellow["00"] != 0:
            cx_blue = int(M_blue["10"]/M_blue["00"])
            cy_blue = int(M_blue["01"]/M_blue["00"])

            cx_yellow = int(M_yellow["10"]/M_yellow["00"])
            cy_yellow = int(M_yellow["01"]/M_yellow["00"])

            center_x = (cx_blue + cx_yellow) // 2
            center_y = (cx_blue + cx_yellow) // 2





    # cv.imshow('before', prev)
    # cv.imshow('after', img)
    cv.imshow('bird', transformed_frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv.destroyAllWindows()