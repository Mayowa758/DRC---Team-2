import cv2 as cv
import numpy as np
from util import *
from configure.undistort_data import *
from colour_detect import *
from misc_detect import *
# from ackermann import *
# import time

# Setting default initial error value
error = 0
# init_GPIO()

# Getting the created arrow templates
# left_arrow_templates, right_arrow_templates = load_templates()

# Function is responsible for getting the video of the camera
while True:
    video = cv.VideoCapture(0)
    if not video.isOpened():
        print("Cannot open camera")
        exit()
    else:
        break
window_width = 640
window_height = 480

# Change the frame resolution and rate of the camera
video.set(cv.CAP_PROP_FRAME_WIDTH, window_width)
video.set(cv.CAP_PROP_FRAME_HEIGHT, window_height)
video.set(cv.CAP_PROP_FPS, 30)


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

# This function is responsible for the maths behind the road detection getting centers, moments etc.
def road_detection(blue_contour, yellow_contour, transformed_frame, frame):
    # Default values so the code doesn't break
    error = 0
    center_x = transformed_frame.shape[1] // 2
    frame_center_x = transformed_frame.shape[1] // 2
    cx_blue = 0
    cx_yellow = 0

    if not blue_contour and not yellow_contour:
        # No contours at all
        print("No blue or yellow contours detected.")
        return error, center_x, cx_blue, cx_yellow

    blue_line = get_largest_contour(blue_contour) if blue_contour else None
    yellow_line = get_largest_contour(yellow_contour) if yellow_contour else None

    M_blue = cv.moments(blue_line) if blue_line is not None else None
    M_yellow = cv.moments(yellow_line) if yellow_line is not None else None
    road_width_estimate = 300

    if M_blue and M_yellow and M_blue['m00'] != 0 and M_yellow['m00'] != 0:
        # Both lines are valid
        cx_blue = int(M_blue['m10'] / M_blue['m00'])
        cy_blue = int(M_blue['m01'] / M_blue['m00'])

        cx_yellow = int(M_yellow['m10'] / M_yellow['m00'])
        cy_yellow = int(M_yellow['m01'] / M_yellow['m00'])

        center_x = (cx_blue + cx_yellow) // 2
        print(center_x)
        cv.line(transformed_frame, (cx_blue, cy_blue), (cx_yellow, cy_yellow), (255, 255, 255), 1)

    elif M_blue and M_blue['m00'] != 0:
        # Only blue line is valid
        cx_blue = int(M_blue['m10'] / M_blue['m00'])
        cy_blue = int(M_blue['m01'] / M_blue['m00'])
        center_x = cx_blue - road_width_estimate # slight left bias
        cv.circle(transformed_frame, (cx_blue, cy_blue), 5, (255, 0, 0), -1)

    elif M_yellow and M_yellow['m00'] != 0:
        # Only yellow line is valid
        cx_yellow = int(M_yellow['m10'] / M_yellow['m00'])
        cy_yellow = int(M_yellow['m01'] / M_yellow['m00'])
        center_x = cx_yellow + road_width_estimate  # slight right bias
        cv.circle(transformed_frame, (cx_yellow, cy_yellow), 5, (0, 255, 255), -1)

    else:
        # Moments exist but no valid area (m00 == 0), skip this frame
        print("Contours found but zero area (m00 == 0). Skipping frame.")

    # Visualize center line and error
    error = frame_center_x - center_x
    cv.line(transformed_frame, (frame_center_x, 0), (frame_center_x, transformed_frame.shape[0]), (0, 0, 255), 2)
    cv.putText(transformed_frame, f"The error is: {error}", (30, 30), cv.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 255), 2)
    return error, center_x, cx_blue, cx_yellow

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
        cv.imshow('finish line', green_mask)
        cv.rectangle(transformed_frame, (x,y), (x + w, y + h), green, 2)
        if h > 10 and w > frame_x * 0.4 and y > frame_y * 0.7:
            print("We made it to the finish!!")
            return True
    return False

# Starting timer right before video capture
# prev_time = time.time()

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
    started = False
    finished = False

    attempts = 20
    while (attempts > 0):
        ret, frame = video.read()
        if not ret or frame is None:
            attempts -= 1
            print("Failed to read frame from camera")
            continue
        else:
            # Applies camera undistortion right before we capture video
            newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (window_width, window_height),
                                                            0, (window_width, window_height))
            h, w = frame.shape[:2]
            mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv.CV_16SC2)
            break
    if (attempts == 0):
            raise RuntimeError("The camera was not able to read after multiple attempts")

    # The video capture of the camera
    while True:
        global error
        # Setup for road detection
        _, img = video.read()
        if not _ or img is None:
            print("Frame capture failed, skipping this frame.")
            continue
        prev = img
        img = cv.remap(img, mapx, mapy, interpolation=cv.INTER_LINEAR)
        img = cv.GaussianBlur(img, (13, 13), 0)
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        transformed_frame = perspective_transform(img)
        transformed_frame_hsv = perspective_transform(hsv_img)
        (blue_contour, yellow_contour) = road_setup(hsv_img, transformed_frame)

        # if GPIO.input(ENABLE_PIN) == GPIO.HIGH:
        #     stop_motors()    # this is for safety (to make sure the car is stopped)
        #     stop_servo()
        #     print("Movement disabled. Waiting for enable switch...")
        #     time.sleep(0.5)
        #     continue

        if finished:
            print("Car has reached finish line! Waiting for 'r' to reset or 'q' to quit")
            while True:
                key = cv.waitKey(1) & 0xFF
                if key == ord('r'):
                    print("Resetting car...")
                    finished = False
                    break
                elif key == ord('q'):
                    print("Exiting program...")
                    # shutdown()
                    video.release()
                    cv.destroyAllWindows()
                    return

        if cv.waitKey(1) & 0xFF == ord(' '):
            started = True

        if not started:
            continue

        # Obtain error for PID detection
        # error, road_center_x, cx_blue, cx_yellow = road_detection(blue_contour, yellow_contour, transformed_frame, hsv_img)
        # error = arrow_detection(transformed_frame, error, road_center_x, hsv_img, cx_blue, cx_yellow)
        error = obstacle_detection(hsv_img, error)
        # print(error)

        # Converting error into steering angle using PID control
        # current_time = time.time()
        # global prev_time
        # dt = current_time - prev_time
        # prev_time = current_time

        # Obtaining steering angle and calculating speed from steering angle
        # control = compute_PID_error(error, dt)
        # print(control)
        # steering_angle = compute_steering_angle(control)
        # speed = calculate_speed(steering_angle)

        # Steering angle and speed implemented on servo motor and DC motors respectively
        # set_servo_angle(steering_angle)
        # set_motor_speed(speed)

        if finish_line(transformed_frame_hsv):
            # stop_motors()
            # stop_servo()
            finished = True
            started = False
            continue

        # cv.imshow('before', prev)
        # cv.imshow('after', img)
        cv.imshow('bird', transformed_frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    # shutdown()
    video.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    road_detect()
