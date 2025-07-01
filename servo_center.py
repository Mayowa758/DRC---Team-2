import pigpio
import time

# Connect to the pigpio daemon
pi = pigpio.pi()

# Define the GPIO pin connected to the servo signal wire
servo_pin = 18

# Set the servo to the center position (1500 us pulse width)
pi.set_servo_pulsewidth(servo_pin, 1500)

# Keep the servo centered for a short duration
time.sleep(2)

# Disconnect from the pigpio daemon
pi.stop()