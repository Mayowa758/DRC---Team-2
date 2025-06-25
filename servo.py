import numpy as np
from gpiozero import AngularServo, PWMOutputDevice, DigitalOutputDevice
import math
import time


#################################### Defining servo and DC motor connection pins ################################################
# Defining servo motor pin
SERVO_PIN = 25
servo = AngularServo(
        pin=SERVO_PIN,
        min_angle=MIN_STEERING_ANGLE,
        max_angle=MAX_STEERING_ANGLE,
        min_pulse_width=PULSE_MIN,  # 0.001
        max_pulse_width=PULSE_MAX   # 0.002
    )
servo.angle = 0