import RPi.GPIO as GPIO
from time import sleep

# Define GPIO pins
DIR_A = 6
PWM_A = 13

DIR_B = 5
PWM_B = 12

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(DIR_A, GPIO.OUT)
GPIO.setup(PWM_A, GPIO.OUT)
GPIO.setup(DIR_B, GPIO.OUT)
GPIO.setup(PWM_B, GPIO.OUT)


GPIO.output(DIR_A, GPIO.HIGH)
GPIO.output(DIR_B, GPIO.HIGH)

# Set up PWM at 1kHz frequency
pwm_a = GPIO.PWM(PWM_A, 1000)
pwm_b = GPIO.PWM(PWM_B, 1000)
pwm_a.start(0)
pwm_b.start(0)

# Control function
def set_motor(dir_pin, pwm_obj, direction, speed_percent):
    GPIO.output(dir_pin, GPIO.HIGH if direction == "forward" else GPIO.LOW)
    pwm_obj.ChangeDutyCycle(speed_percent)

try:
    print("Motors forward")
    set_motor(DIR_A, pwm_a, "forward", 100)
    set_motor(DIR_B, pwm_b, "forward", 100)
    sleep(2)

    print("Motors backward")
    set_motor(DIR_A, pwm_a, "backward", 100)
    set_motor(DIR_B, pwm_b, "backward", 100)
    sleep(2)
    sleep(5)
    print("Motors stop")
    pwm_a.ChangeDutyCycle(0)
    pwm_b.ChangeDutyCycle(0)

finally:
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
