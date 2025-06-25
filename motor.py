import numpy as np
from gpiozero import AngularServo, PWMOutputDevice, DigitalOutputDevice
import math
import time


#################################### Defining servo and DC motor connection pins ################################################
# Defining servo motor pin
SERVO_PIN = 25

# Defining DC motor pins
# Left motor

LEFT_DIR = 29
LEFT_PWM = 33

# Right motor
RIGHT_DIR = 31
RIGHT_PWM = 32

GPIO_INITIALIZED = False



##################################### Computing speed and angle constants ########################################################
# Pure pursuit constants
LOOKAHEAD_DISTANCE = 30
SCALING_FACTOR = 0.5

# Angle and Pulse constants
MAX_STEERING_ANGLE = 30
MIN_STEERING_ANGLE = -30
PULSE_MIN = 0.001
PULSE_MAX = 0.002

################################### Connecting the servo motor and DC motor pins to the Raspberry Pi ##############################
# GPIO.setmode(GPIO.BCM)
# GPIO.setwarnings(False)

# Function to set up the GPIO
def init_GPIO():
    global GPIO_INITIALIZED, left_pwm, right_pwm, servo
    if GPIO_INITIALIZED:
        return left_pwm, right_pwm, servo
    # GPIO.setmode(GPIO.BCM)
    # GPIO.setup(LEFT_DIR, GPIO.OUT)
    # GPIO.setup(RIGHT_DIR, GPIO.OUT)
    # GPIO.setup(LEFT_PWM, GPIO.OUT)
    # GPIO.setup(RIGHT_PWM, GPIO.OUT)

    # Initialise PWM
    # left_pwm = GPIO.PWM(LEFT_PWM, 1000)  # 1kHz frequency
    # right_pwm = GPIO.PWM(RIGHT_PWM, 1000)
    # left_pwm.start(0)
    # right_pwm.start(0)
    left_pwm = PWMOutputDevice(LEFT_PWM)
    right_pwm = PWMOutputDevice(RIGHT_PWM)
    left_dir = DigitalOutputDevice(LEFT_DIR)
    right_dir = DigitalOutputDevice(RIGHT_DIR)

    left_dir.on()
    right_dir.on()

    # Initialise AngularServo (now using seconds)
    servo = AngularServo(
        pin=SERVO_PIN,
        min_angle=MIN_STEERING_ANGLE,
        max_angle=MAX_STEERING_ANGLE,
        min_pulse_width=PULSE_MIN,  # 0.001
        max_pulse_width=PULSE_MAX   # 0.002
    )
    servo.angle = 0

    GPIO_INITIALIZED = True

    return left_pwm, right_pwm, left_dir, right_dir, servo


left_pwm, right_pwm, left_dir, right_dir, servo = init_GPIO()

# This function allows the steering angle calculated to be actuated on the servo
def set_servo_angle(angle):
    # This function maps the steering angle to microseconds (or duty cycle) - servos understand PWM pulses, not angles
    angle = max(MIN_STEERING_ANGLE, min(MAX_STEERING_ANGLE, angle))
    servo.angle = angle
    # pulse_width = int(np.interp(angle, [MIN_STEERING_ANGLE, MAX_STEERING_ANGLE], [PULSE_MIN, PULSE_MAX]))
    # pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)



# This function allows the speed calculated to be actuated on the DC motors
def set_motor_speed(speed):
    duty = max(0.0, min(1.0, speed))
    left_pwm.value = duty
    right_pwm.value = duty

# This function stops the servo but doesn't turn it off
def stop_servo():
    servo.angle = 0

# This function turns the servo motor off
# def turn_off_servo():
#     # servo_pwm.set_servo_pulsewidth(SERVO_PIN, 0)
#     # servo_pwm.stop()
#     servo.angle = 0
#     time.sleep(0.5)

# This function stops the motor but doesn't turn it off
def stop_motors():
    # left_pwm.ChangeDutyCycle(0)
    # right_pwm.ChangeDutyCycle(0)
    left_pwm.off()
    right_pwm.off()

# This function turns the motor off
# def turn_off_motors():
#     # left_pwm.stop()
#     # right_pwm.stop()
#     stop_motors()
#     left_pwm.close()
#     right_pwm.close()
#     left_dir.close()
#     right_dir.close()

# This function resets all GPIO pins to their default input mode when a program exits, preventing potential issues with connected components
# def cleanup_GPIO():
#     GPIO.cleanup()

# Shuts down the motors once program is quit
def shutdown():
    print("Shutting down...")
    # try:
    stop_motors()

    stop_servo()
    time.sleep(1.0)
    servo.value = None
    servo.close()

    left_pwm.close()
    right_pwm.close()
    left_dir.close()
    right_dir.close()
    # turn_off_servo()  # set servo to 0 angle, then detach
    # finally:
    #     cleanup_GPIO()

def test():
    servo.angle = 0
    while True:
        set_motor_speed(0.2)
        time.sleep(3.0)
        break

    shutdown()

if __name__ == "__main__":
    test()


