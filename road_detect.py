import cv2 as cv
import numpy as np
from util import *
from configure.undistort_data import *
from colour_detect import *
from misc_detect import *
from ackermann import *
import time

# Initialise the video reading device and the resolution of video
video = cv.VideoCapture(0)
window_width = 640
window_height = 480

# Change the frame resolution and rate of the camera
video.set(cv.CAP_PROP_FRAME_WIDTH, window_width)
video.set(cv.CAP_PROP_FRAME_HEIGHT, window_height)
video.set(cv.CAP_PROP_FPS, 30)

# Setting default initial error value
error = 0

# Getting the created arrow templates
left_arrow_templates, right_arrow_templates = load_templates()

# This function creates a road mask which is combination of blue and yellow
def road_mask(blue, yellow):
    new_mask = blue | yellow
    return new_mask

# This function is responsible for changing the normal view of camera to a birds-eye perspective
def perspective_transform(img):

    # These values will need to change
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

# This function gets the largest contours extracted from camera view
def get_largest_contour(contours):
    if contours:
        return max(contours, key=cv.contourArea)
    else :
      return None

# This function is responsible for the maths behind the road detection getting centers, moments etc.
def road_detection(blue_contour, yellow_contour, transformed_frame, frame):
    # Default values so the code doesn't break
    error = 0
    center_x = transformed_frame.shape[1] // 2

    # If valid blue and yellow contours peform the road detection functionality
    if blue_contour and yellow_contour:
        blue_line = get_largest_contour(blue_contour)
        yellow_line = get_largest_contour(yellow_contour)

        # Uses moments to find the centers of the blue and yellow line
        M_blue = cv.moments(blue_line)
        M_yellow = cv.moments(yellow_line)

        if M_blue['m00'] != 0 and M_yellow['m00'] != 0:
            cx_blue = int(M_blue['m10']/M_blue['m00'])
            cy_blue = int(M_blue['m01']/M_blue['m00'])

            cx_yellow = int(M_yellow['m10']/M_yellow['m00'])
            cy_yellow = int(M_yellow['m01']/M_yellow['m00'])

            center_x = (cx_blue + cx_yellow) // 2
            center_y = (cy_blue + cy_yellow) // 2

            frame_widthx = transformed_frame.shape[1]
            frame_center_x = frame_widthx // 2
            error = frame_center_x - center_x
            # print(error)

            # Line to show the Calculated Center
            cv.line(transformed_frame, (cx_blue, cy_blue), (cx_yellow, cy_yellow),
                    (255, 255, 255), 1)
            # Line to show the Frame center
            cv.line(transformed_frame, (frame_center_x, 0), (frame_center_x, transformed_frame.shape[0]),
                    (0, 0, 255), 2)
            # Showcases the error
            cv.putText(transformed_frame, f"The error is: {error}", (30,30), cv.FONT_HERSHEY_COMPLEX, 0.7,
                    (0, 255, 255), 2)
            return (error, center_x)
    else:
        return (error, center_x)
        # Backup code if one or no lines are detected
        # elif yellow_contour and not blue_contour:
        #     error = frame_center_x + 50
        # elif blue_contour and not yellow_contour:
        #     error = frame_center_x - 50
        # else:
        #     error += 25

# Function that detects the finish line and gets the car to stop
def finish_line(transformed_frame):
    green_range = get_limits(green)
    green_mask = get_mask(transformed_frame, green_range, kernel)
    green_contour, _ = cv.findContours(green_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    frame_x = transformed_frame.shape[1]
    frame_y = transformed_frame.shape[0]
    if green_contour:
        green_area = get_largest_contour(green_contour)
        x, y, w, h = cv.boundingRect(green_area)
        if h > 20 and w > frame_x * 0.6 and y > frame_y * 0.7:
            print("We made it to the finish!!")
            stop_motor()

# Starting timer right before video capture
prev_time = time.time()

# Function is responsible for setting up masks and birds eye transformation for effective road detection
def road_setup(hsv_img, transformed_frame):
    # Get colour range
    blue_range = get_limits(blue)
    yellow_range = get_limits(yellow)

    # Create corresponding masks
    blue_mask = get_mask(hsv_img, blue_range, kernel)
    yellow_mask = get_mask(hsv_img, yellow_range, kernel)
    drive_mask = road_mask(blue_mask, yellow_mask)

    # Bird's eye transformation
    hsv_img_bv = cv.cvtColor(transformed_frame, cv.COLOR_BGR2HSV)
    blue_mask_bv = get_mask(hsv_img_bv, blue_range, kernel)
    yellow_mask_bv = get_mask(hsv_img_bv, yellow_range, kernel)
    drive_mask_bv = road_mask(blue_mask_bv, yellow_mask_bv)

    cv.imshow('drive_mask_bv', drive_mask_bv)
    # cv.imshow('blue', blue_mask)
    # cv.imshow('yellow', yellow_mask)

    # Gets contours of blue and yellow masks respectively
    blue_contour, _ = cv.findContours(blue_mask_bv, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    yellow_contour, _ = cv.findContours(yellow_mask_bv, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return (blue_contour, yellow_contour)

# MAIN RUNNING FUNCTION
def road_detect():
    # Applies camera undistortion right before we capture video
    ret, frame = video.read()
    if not ret or frame is None:
        raise RuntimeError("Failed to read frame from camera")
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (window_width, window_height),
                                                    0, (window_width, window_height))
    h, w = frame.shape[:2]
    mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv.CV_16SC2)

    # The video capture of the camera
    while True:
        global error
        # Setup for road detection
        _, img = video.read()
        prev = img
        img = cv.remap(img, mapx, mapy, interpolation=cv.INTER_LINEAR)
        img = cv.GaussianBlur(img, (13, 13), 0)
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        transformed_frame = perspective_transform(img)
        (blue_contour, yellow_contour) = road_setup(hsv_img, transformed_frame)

        # Obtain error for PID detection
        error, road_center_x = road_detection(blue_contour, yellow_contour, transformed_frame, hsv_img)
        error = arrow_detection(transformed_frame, error, road_center_x)
        error = obstacle_detection(hsv_img, error)

        # Converting error into steering angle using PID control
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Obtaining steering angle and calculating speed from steering angle
        steering_angle = convert_PID_error_to_steering_angle(error, dt)
        speed = calculate_speed(steering_angle)

        # Steering angle and speed implemented on servo motor and DC motors respectively
        set_servo_angle(steering_angle)
        set_motor_speed(speed)
        finish_line(transformed_frame)
        # cv.imshow('before', prev)
        # cv.imshow('after', img)
        cv.imshow('bird', transformed_frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    close_servo()
    close_motor()
    cleanup_GPIO()
    video.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    road_detect()