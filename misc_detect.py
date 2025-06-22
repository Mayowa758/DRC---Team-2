import cv2 as cv
from colour_detect import *
from road_detect import *
from util import *

# Function that detects a black arrow on the ground and turns in the following direction
def arrow_detection(frame, error):
    # Some value for correcting PID
    # correction_factor = 100
    # gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    # gray_frame = gray_frame[int(gray_frame.shape[0] * 0.6):, :] # Crops the frame
    # best_left_score = 0
    # best_right_score = 0

    # # Match with left templates
    # for tmpl in left_arrow_templates:
    #     tmpl_resized = cv.resize(tmpl, (tmpl.shape[1], tmpl.shape[0]))  # Ensure size consistency
    #     res = cv.matchTemplate(gray_frame, tmpl_resized, cv.TM_CCOEFF_NORMED)
    #     _, max_val, _, _ = cv.minMaxLoc(res)
    #     best_left_score = max(best_left_score, max_val)

    # # Match with right templates
    # for tmpl in right_arrow_templates:
    #     tmpl_resized = cv.resize(tmpl, (tmpl.shape[1], tmpl.shape[0]))
    #     res = cv.matchTemplate(gray_frame, tmpl_resized, cv.TM_CCOEFF_NORMED)
    #     _, max_val, _, _ = cv.minMaxLoc(res)
    #     best_right_score = max(best_right_score, max_val)

    # threshold = 0.5

    # if best_left_score > threshold or best_right_score > threshold:
    #     if best_left_score > best_right_score:
    #         error -= correction_factor
    #         print("Left arrow detected")
    #     else:
    #         error += correction_factor
    #         print("Right arrow detected")
    # else:
    #     print("No confident arrow match")
    # return error
    #Backup arrow detection
    correction_factor = 100;

    # Find contours of black arrow
    black_range = get_limits(black)
    arrow_mask = get_mask(frame, black_range, kernel)
    arrow_contours, _ = cv.findContours(arrow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not arrow_contours:
        return error
    arrow_area = get_largest_contour(arrow_contours)
    arrow_M = cv.moments(arrow_area)

    # Calculate the centroid of the black arrow
    arrow_Mx = int(arrow_M['m10']/arrow_M['m00'])
    if arrow_M['m00'] == 0:
        return error

    # center of the camera frame
    frame_center = frame.shape[1] // 2

    # Perform the error correction
    if arrow_Mx > frame_center:
        error -= correction_factor
        print("left")
    else:
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

