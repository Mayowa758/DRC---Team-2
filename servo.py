import numpy as np
from gpiozero import AngularServo, PWMOutputDevice, DigitalOutputDevice
import math
import time


#################################### Defining servo and DC motor connection pins ################################################
# Defining servo motor pin
SERVO_PIN = 12
servo = AngularServo(
        pin=SERVO_PIN,
        min_angle=-50,
        max_angle=50,
        min_pulse_width=0.001,  # 0.001
        max_pulse_width=0.002   # 0.002
    )
servo.angle = 0