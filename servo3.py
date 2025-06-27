from gpiozero import AngularServo
from time import sleep

SERVO_PIN =18

servo = AngularServo(SERVO_PIN,
                     min_angle=-50, max_angle=50,
                     min_pulse_width=0.00107,
                     max_pulse_width=0.00197)

try:
    while True:
        # Move to ~-50
        servo.angle = -50
        print("Position: ~-50")
        sleep(2)

        # Move to center (0)
        servo.angle = 0.0
        print("Position: 0 (center)")
        sleep(2)

        # Move to ~+50
        servo.angle = 50
        print("Position: ~+50")
        sleep(2)

except KeyboardInterrupt:
    print("\nTest stopped by user.")
    servo.detach()