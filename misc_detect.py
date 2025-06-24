import cv2 as cv
from colour_detect import *
from util import *

lower_black = np.array([100, 25, 165])
upper_black = np.array([130, 50, 179])

# Function that detects a black arrow on the ground and turns in the following direction
def arrow_detection(frame, error, road_center_x):
    correction_factor = 100

    # Find contours of black arrow
    black_range = get_limits(black)
    arrow_mask = get_mask(frame, black_range, kernel)
    arrow_contours, _ = cv.findContours(arrow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Initial checks to see if arrow is valid
    if not arrow_contours or road_center_x is None:
        return error
    arrow_area = get_largest_contour(arrow_contours)
    arrow_M = cv.moments(arrow_area)

    if arrow_M['m00'] == 0:
        return error
    if cv.contourArea(arrow_area) < 1000:
        print("Nothing detected")
        return error

    # Calculate the centroid of the black arrow
    arrow_Mx = int(arrow_M['m10']/arrow_M['m00'])

    # Visualisation for Debugging
    cv.circle(frame, (arrow_Mx, frame.shape[0] // 2), 5, (0, 255, 0), -1)  # Arrow center
    cv.imshow('arrow', arrow_mask)

    # Perform the error correction
    if arrow_Mx > road_center_x:
        error -= correction_factor
        print("left")
    elif arrow_Mx < road_center_x:
        error += correction_factor
        print("right");

    return error

# Function that detects purple obstacles and increases or decreases the PID error accordingly
def obstacle_detection(hsv_img, error):
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

    # Get center of the camera frame
    frame_center = hsv_img.shape[1] // 2

    # Perform the error correction
    if object_Mx > frame_center:
        error -= correction_factor
        print("left")
    else:
        error += correction_factor
        print("right")
    return error

