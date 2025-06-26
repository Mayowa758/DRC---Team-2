from gpiozero import Servo
from time import sleep

SERVO_PIN = 18  # GPIO18 (physical pin 12)

# Create servo object with accurate pulse width limits
servo = Servo(
    SERVO_PIN,
    min_pulse_width=0.00107,  # 1070 �s
    max_pulse_width=0.00197   # 1970 �s
)

try:
    while True:
        # Move to ~-50�
        servo.value = -1.0
        print("Position: ~-50�")
        sleep(2)

        # Move to center (0�)
        servo.value = 0.0
        print("Position: 0� (center)")
        sleep(2)

        # Move to ~+50�
        servo.value = 1.0
        print("Position: ~+50�")
        sleep(2)

except KeyboardInterrupt:
    print("\nTest stopped by user.")