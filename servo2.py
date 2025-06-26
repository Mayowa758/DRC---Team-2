import RPi.GPIO as GPIO
from time import sleep

# Set GPIO mode and warnings
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Define the PWM pin
pwm_gpio = 12
frequency = 50  # Hz
GPIO.setup(pwm_gpio, GPIO.OUT)
pwm = GPIO.PWM(pwm_gpio, frequency)
pwm.start(0)

# Function to convert angle to duty cycle
def angle_to_percent(angle):
    if angle > 180 or angle < 0:
        return False
    start = 2.5
    end = 12.5
    ratio = (end - start) / 180
    angle_as_percent = angle * ratio
    return start + angle_as_percent

# Main loop to control the servo
try:
    while True:
        # Move to 0 degrees
        pwm.ChangeDutyCycle(angle_to_percent(0))
        sleep(1)

        # Move to 90 degrees
        pwm.ChangeDutyCycle(angle_to_percent(90))
        sleep(1)

        # Move to 180 degrees
        pwm.ChangeDutyCycle(angle_to_percent(180))
        sleep(1)

except KeyboardInterrupt:
    # Clean up GPIO on program exit
    pwm.stop()
    GPIO.cleanup()