import cv2 as cv
from colour_detect import *
from util import *

lower_black = np.array([100, 25, 165])
upper_black = np.array([130, 50, 179])

# Function that detects a black arrow on the ground and turns in the following direction
def arrow_detection(frame, error, road_center_x, hsv_img, cx_blue, cx_yellow):
    correction_factor = 100

    # Find contours of black arrow
    black_range = get_limits(black)
    arrow_mask = get_mask(frame, black_range, kernel)
    arrow_contours, _ = cv.findContours(arrow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # Initial checks to see if arrow is valid
    if not arrow_contours or road_center_x is None:
        # print("No Valid arrow")
        return error
    arrow_area = get_largest_contour(arrow_contours)
    arrow_M = cv.moments(arrow_area)
    
    if arrow_M['m00'] == 0:
        return error
    if cv.contourArea(arrow_area) < 1000:
        # print("Nothing detected")
        return error

    # Calculate the centroid of the black arrow
    arrow_Mx = int(arrow_M['m10']/arrow_M['m00'])

    if cx_blue is None and cx_yellow is None:
        if arrow_Mx > cx_blue or arrow_Mx < cx_yellow:
            return error

    # Visualisation for Debugging
    cv.circle(frame, (arrow_Mx, frame.shape[0] // 2), 5, (0, 255, 0), -1)  # Arrow center
    # cv.imshow('arrow', arrow_mask)

    # Perform the error correction
    if arrow_Mx > road_center_x:
        error -= correction_factor
        # print("left")
    elif arrow_Mx < road_center_x:
        error += correction_factor
        # print("right")

    return error

# Function that detects purple obstacles and increases or decreases the PID error accordingly
def obstacle_detection(transformed_frame_hsv, error):
    correction_factor = 175

    # Get mask and contours for purple obstacle
    purple_range = get_limits(purple)
    purple_mask = get_mask(transformed_frame_hsv, purple_range, kernel)
    contours, _ = cv.findContours(purple_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    if not contours:
        return error

    object_area = get_largest_contour(contours)
    M = cv.moments(object_area)

    if M['m00'] == 0:
        return error  # Avoid division by zero

    # Now safe to calculate centroid
    cx = int(M["m10"] / M["m00"])
    frame_center = transformed_frame_hsv.shape[1] // 2

    # Apply error correction based on object location
    if cx > frame_center:
        error += correction_factor
        # print("Obstacle on RIGHT → steer LEFT → error +=", correction_factor)
    else:
        error -= correction_factor
        # print("Obstacle on LEFT → steer RIGHT → error -=", correction_factor)

    return error

