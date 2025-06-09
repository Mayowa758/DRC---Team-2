import cv2 as cv
import numpy as np
import time
import pigpio
import Rpi.GPIO as GPIO
from util import get_limits
from configure.undistort_data import *
from colour_detect import *
# from gpiozero import AngularServo

# Angle and Pulse constants
MAX_STEERING_ANGLE = 30
MIN_STEERING_ANGLE = -30
PULSE_MIN = 1000
PULSE_MAX = 2000

# PID constants
KP = 0.4    # proportional constant
KI = 0.01   # integral constant
KD = 0.2    # derivative constant

# PID state
integral = 0
last_error = 0

# Integral limit to prevent windup
INTEGRAL_MAX = 100
INTEGRAL_MIN = -100


# Connecting the servo
SERVO_PIN = 18
pi = pigpio.pi()

# Connecting the DC motors
# Left motor
LEFT_IN1 = 23
LEFT_IN2 = 24
LEFT_EN = 18    # PWM pin

# Right motor
RIGHT_IN1 = 27
RIGHT_IN2 = 22
RIGHT_EN = 13   # PWM pin

GPIO.setmode(GPIO.BCM)

# Setup direction pins
GPIO.setup(LEFT_IN1, GPIO.OUT)
GPIO.setup(LEFT_IN2, GPIO.OUT)
GPIO.setup(RIGHT_IN1, GPIO.OUT)
GPIO.setup(RIGHT_IN2, GPIO.OUT)

# Setup PWM pins
GPIO.setup(LEFT_EN, GPIO.OUT)
GPIO.setup(RIGHT_EN, GPIO.OUT)

# Set direction: both motors forward
GPIO.output(LEFT_IN1, GPIO.HIGH)
GPIO.output(LEFT_IN2, GPIO.LOW)

GPIO.output(RIGHT_IN1, GPIO.HIGH)
GPIO.output(RIGHT_IN2, GPIO.LOW)

# Initialise PWM
left_pwm = GPIO.PWM(LEFT_EN, 1000)  # 1kHz frequency
right_pwm = GPIO.PWM(RIGHT_EN, 1000)
left_pwm.start(0)   # Start with 0% duty cycle (stopped)
right_pwm.start(0)


# from colour_detect import mask_blue, mask_yellow
video = cv.VideoCapture(0)
window_width = 640
window_height = 480

# This function creates a road mask which is combination of blue and yellow
def road_mask(blue, yellow):
    new_mask = blue | yellow
    return new_mask


# This function is responsible for changing the normal view to a birds-eye perspective
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

# This function gets the largest contours extracted from camera view
def get_largest_contour(contours):
    if contours:
        return max(contours, key=cv.contourArea)
    else :
      return None
    
# This function converts the PID error into a steering angle
def convert_PID_error_to_steering_angle(error, dt):
    global integral, last_error

    integral += error * dt
    integral = max(min(integral, INTEGRAL_MAX), INTEGRAL_MIN)
    derivative = (error - last_error) / dt
    last_error = error

    control = KP * error + KI * integral + KD * derivative

    # Clamp angle to (allowed) range
    steering_angle = max(min(control, MAX_STEERING_ANGLE), MIN_STEERING_ANGLE)

    return steering_angle

# This function allows the steering angle calculated to be actuated on the servo
def set_servo_angle(angle):
    # This function maps the steering angle to microseconds (or duty cycle) - servos understand PWM pulses, not angles
    pulse_width = int(np.interp(angle, [MIN_STEERING_ANGLE, MAX_STEERING_ANGLE], [PULSE_MIN, PULSE_MAX]))
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)


##################### Nick's code for setting the servo angle ##########################
# # Changing this during testing period - The minimum angle required to slow down the back motors
# MINIMUM_THRESHOLD = 18


# # You should change the first argument to the pin where the servo is connected to in the pi.
# # The 2nd and 3rd arguments are assuming we have a large servo motor.
# # By default, the angle range is from -90 to 90 degrees


# servo = AngularServo(25, min_pulse_width=0.0005, max_pulse_width=0.0025)

# class MotorDrive():
#     def map_steering_angle_to_servo(angle_from_pid):
#         while (True):
#             servo.angle = angle_from_pid
#             print(f"Steering angle set to: {angle_from_pid: .2f}°")
#             sleep(0.1)
#########################################################################################

# This function calculates the speed of the wheels based on the steering angle
def calculate_speed(steering_angle, max_speed=1.0, min_speed=0.4):
    angle = abs(steering_angle)

    speed = max_speed - (angle / MAX_STEERING_ANGLE) * (max_speed - min_speed) 
    return speed

# This function allows the speed calculated to be actuated on the DC motors
def set_motor_speed(speed):
    duty = speed * 100
    left_pwm.ChangeDutyCycle(duty)
    right_pwm.ChangeDutyCycle(duty)

# Change the frame rate of the camera
video.set(cv.CAP_PROP_FRAME_WIDTH, window_width)
video.set(cv.CAP_PROP_FRAME_HEIGHT, window_height)
video.set(cv.CAP_PROP_FPS, 30)

# Applies camera undistortion right before we turn it on
ret, frame = video.read()
if not ret or frame is None:
    raise RuntimeError("Failed to read frame from camera")
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (window_width, window_height),
                                                 0, (window_width, window_height))
h, w = frame.shape[:2]
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv.CV_16SC2)

# Setting default error value
error = 0

# Starting timer right before video capture
prev_time = time.time()

# The video capture of the camera
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

    # Gets contours of blue and yellow masks respectively
    blue_contour, _ = cv.findContours(blue_mask_bv, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    yellow_contour, _ = cv.findContours(yellow_mask_bv, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

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
            print(error)

            # Converting error into steering angle using PID control
            current_time = time.time()
            dt = current_time - prev_time
            prev_time = current_time

            # Obtaining steering angle
            steering_angle = convert_PID_error_to_steering_angle(error, dt)

            # Calculating speed from steering angle
            speed = calculate_speed(steering_angle)

            # Steering angle and speed implemented on servo motor and DC motors respectively
            set_servo_angle(steering_angle)
            set_motor_speed(speed)

            # Line to show the Calculated Center
            cv.line(transformed_frame, (cx_blue, cy_blue), (cx_yellow, cy_yellow),
                    (255, 255, 255), 1)
            # Line to show the Frame center
            cv.line(transformed_frame, (frame_center_x, 0), (frame_center_x, transformed_frame.shape[0]),
                    (0, 0, 255), 2)
            # Showcases the error
            cv.putText(transformed_frame, f"The error is: {error}", (30,30), cv.FONT_HERSHEY_COMPLEX, 0.7,
                    (0, 255, 255), 2)

    # cv.imshow('before', prev)
    # cv.imshow('after', img)
    cv.imshow('bird', transformed_frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

pi.set_servo_pulsewidth(SERVO_PIN, 0)
pi.stop()
left_pwm.stop()
right_pwm.stop()
GPIO.cleanup()
video.release()
cv.destroyAllWindows()