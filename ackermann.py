import pigpio
import Rpi.GPIO as GPIO
import numpy as np
# from gpiozero import AngularServo

# Angle and Pulse constants
MAX_STEERING_ANGLE = 30
MIN_STEERING_ANGLE = -30
PULSE_MIN = 1000
PULSE_MAX = 2000

# PID constants
# These values will need to be adjusted
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

# This function turns the servo motor off
def close_servo():
    pi.set_servo_pulsewidth(SERVO_PIN, 0)
    pi.stop()

# This function stops the motor but doesn't turn it off
def stop_motor():
    left_pwm.ChangeDutyCycle(0)
    right_pwm.ChangeDutyCycle(0)

# This function turns the motor off
def close_motor():
    left_pwm.stop()
    right_pwm.stop()

# This function resets all GPIO pins to their default input mode when a program exits, preventing potential issues with connected components
def cleanup_GPIO():
    GPIO.cleanup()
