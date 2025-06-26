import pigpio
import time
# import RPi.GPIO as GPIO
# GPIO.setmode(GPIO.BCM)

# Connect to the pigpio daemon
pi = pigpio.pi()

# Define the GPIO pin connected to the servo
servo_pin = 18

# Set the servo pin to output mode
pi.set_mode(servo_pin, pigpio.OUTPUT)

# Function to move the servo to a specific angle (pulse width in microseconds)
def set_servo_angle(angle):
    pulse_width = 1000 + (angle / 180.0) * 1000  # Convert angle to pulse width
    pi.set_servo_pulsewidth(servo_pin, pulse_width)

# Example usage:
set_servo_angle(0)
time.sleep(1)
set_servo_angle(90)
time.sleep(1)
set_servo_angle(180)
time.sleep(1)
set_servo_angle(90)
time.sleep(1)

# Stop the servo
pi.set_servo_pulsewidth(servo_pin, 0)

# Disconnect from the pigpio daemon
pi.stop()