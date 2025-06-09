from gpiozero import AngularServo
from time import sleep


# Changing this during testing period - The minimum angle required to slow down the back motors
MINIMUM_THRESHOLD = 18


# You should change the first argument to the pin where the servo is connected to in the pi.
# The 2nd and 3rd arguments are assuming we have a large servo motor.
# By default, the angle range is from -90 to 90 degrees


servo = AngularServo(25, min_pulse_width=0.0005, max_pulse_width=0.0025)

class MotorDrive():
    def map_steering_angle_to_servo(angle_from_pid):
        while (True):
            servo.angle = angle_from_pid
            print(f"Steering angle set to: {angle_from_pid: .2f}°")
            sleep(0.1)