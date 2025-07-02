import RPi.GPIO as GPIO
import numpy as np
# from gpiozero import AngularServo
import pigpio
import math
import time


#################################### Defining servo and DC motor connection pins ################################################
# Defining servo motor pin
SERVO_PIN = 18

# Defining DC motor pins
# Left motor
LEFT_DIR = 6
LEFT_PWM = 13    

# Right motor
RIGHT_DIR = 5
RIGHT_PWM = 12

GPIO_INITIALIZED = False

######################################### PID constants and global variables #####################################################
# PID constants
# These values will need to be adjusted
KP = 0.4    # proportional constant (0.4)
KI = 0   # integral constant (0.01)
KD = 0    # derivative constant (0.2)

# PID state
integral = 0
last_error = 0

# Integral limit to prevent windup
INTEGRAL_MAX = 100
INTEGRAL_MIN = -100

##################################### Computing speed and angle constants ########################################################
# Pure pursuit constants
LOOKAHEAD_DISTANCE = 250
SCALING_FACTOR = 0.5

# Constants for servo motor (angle and pulse constants)
MAX_STEERING_ANGLE = 50
MIN_STEERING_ANGLE = -50
PULSE_MIN = 900
PULSE_MAX = 2100
CENTRE_PULSE = 1500

################################### Connecting the servo motor and DC motor pins to the Raspberry Pi ##############################
# Function to set up the GPIO
def init_GPIO():
    global GPIO_INITIALIZED, left_pwm, right_pwm, servo
    if GPIO_INITIALIZED:
        return left_pwm, right_pwm, servo

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # Setup direction and PWM pins
    GPIO.setup(LEFT_DIR, GPIO.OUT)
    GPIO.setup(RIGHT_DIR, GPIO.OUT)
    GPIO.setup(LEFT_PWM, GPIO.OUT)
    GPIO.setup(RIGHT_PWM, GPIO.OUT)

    # Initialise direction and PWM pins
    GPIO.output(LEFT_DIR, GPIO.HIGH)
    GPIO.output(RIGHT_DIR, GPIO.HIGH)
    
    left_pwm = GPIO.PWM(LEFT_PWM, 100)  # 1kHz frequency
    right_pwm = GPIO.PWM(RIGHT_PWM, 100)
    left_pwm.start(0)
    right_pwm.start(0)

    # Initialise AngularServo (now using seconds)
    # servo = AngularServo(
    #     pin=SERVO_PIN,
    #     min_angle=MIN_STEERING_ANGLE,
    #     max_angle=MAX_STEERING_ANGLE,
    #     min_pulse_width=PULSE_MIN,  # 0.0009
    #     max_pulse_width=PULSE_MAX   # 0.0021
    # )
    # servo.angle = 0
    servo = pigpio.pi()
    if not servo.connected:
        raise IOError("Cannot connect to pigpio daemon!")
    servo.set_servo_pulsewidth(SERVO_PIN, CENTRE_PULSE)
    
    GPIO_INITIALIZED = True
    return left_pwm, right_pwm, servo

#################################################### FUNCTIONS ####################################################################
# This function converts the PID error into a steering angle
def compute_PID_error(error, dt):
    global integral, last_error

    if (abs(error) > 300):



    integral += error * dt
    integral = max(min(integral, INTEGRAL_MAX), INTEGRAL_MIN)
    derivative = (error - last_error) / dt
    last_error = error

    control = KP * error + KI * integral + KD * derivative
    
    return control

# This function computes the steering angle from the adjusted error/control value
def compute_steering_angle(control):
    # Avoiding small unnecessary conections (which will jitter the servo)
    if abs(control) < 5:
        return 0
        
    # Compute geometric angle using Pure Pursuit
    desired_angle_rad = math.atan2(control, LOOKAHEAD_DISTANCE)
    desired_angle_deg = math.degrees(desired_angle_rad)

    # Smooth the response using tanh for stability
    # steering_angle = MAX_STEERING_ANGLE * np.tanh(desired_angle_deg / SCALING_FACTOR)
    
    # Clamp angle to (allowed) range
    steering_angle = -max(min(desired_angle_deg, MAX_STEERING_ANGLE), MIN_STEERING_ANGLE)
    # print(steering_angle)
    return steering_angle

# This function calculates the speed of the wheels based on the steering angle
def calculate_speed(steering_angle, max_speed=0.85, min_speed=0.4):
    angle = abs(steering_angle)

    speed = max_speed - (angle / MAX_STEERING_ANGLE) * (max_speed - min_speed) 
    return speed


left_pwm, right_pwm, servo = init_GPIO()


# This function allows the steering angle calculated to be actuated on the servo
def set_servo_angle(angle):
    # This function maps the steering angle to microseconds (or duty cycle) - servos understand PWM pulses, not angles
    # angle = max(MIN_STEERING_ANGLE, min(MAX_STEERING_ANGLE, angle))
    # servo.angle = angle
    angle = max(MIN_STEERING_ANGLE, min(MAX_STEERING_ANGLE, angle))
    pulse_width = int(np.interp(angle, [MIN_STEERING_ANGLE, MAX_STEERING_ANGLE], [PULSE_MIN, PULSE_MAX]))
    servo.set_servo_pulsewidth(SERVO_PIN, pulse_width)


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

# This function allows the speed calculated to be actuated on the DC motors
def set_motor_speed(speed):
    duty = max(0.0, min(1.0, speed)) * 100
    left_pwm.ChangeDutyCycle(duty)
    right_pwm.ChangeDutyCycle(duty)


# This function stops the servo but doesn't turn it off
def stop_servo():
    # servo.angle = 0
    servo.set_servo_pulsewidth(SERVO_PIN, 0)
    
# # This function turns the servo motor off
# def turn_off_servo():
#     # servo.set_servo_pulsewidth(SERVO_PIN, 0)
#     servo.stop()
#     # servo.angle = 0
#     # time.sleep(1.0)
#     # servo.value = None
#     # servo.close()

# This function stops the motor but doesn't turn it off
def stop_motors():
    left_pwm.ChangeDutyCycle(0)
    right_pwm.ChangeDutyCycle(0)

# # This function turns the motor off
# def turn_off_motors():
#     left_pwm.stop()
#     right_pwm.stop()

# # This function resets all GPIO pins to their default input mode when a program exits, preventing potential issues with connected components
# def cleanup_GPIO():
#     GPIO.cleanup()

def shutdown():
    print("Shutting down...")
    servo.set_servo_pulsewidth(SERVO_PIN, CENTRE_PULSE)
    stop_motors()
    stop_servo()
    left_pwm.stop()
    right_pwm.stop()
    servo.stop()
    # turn_off_motors()
    # turn_off_servo()
    GPIO.cleanup()
    print("Successfully shut down!")
