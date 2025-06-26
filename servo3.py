from gpiozero import Servo
from time import sleep

SERVO_PIN = 18  # GPIO18, physical pin 12

servo = Servo(
    SERVO_PIN,
    min_pulse_width=0.0009,  # 0.5 ms
    max_pulse_width=0.0021   # 2.5 ms
)

try:
    while True:
        servo.value = -0.25
        print("Angle: -50�")
        sleep(2)
        
        servo.value = 0
        print("Angle: 0�")
        sleep(2)
        
        servo.value = 0.25
        print("Angle: 50�")
        sleep(2)

except KeyboardInterrupt:
    print("Program stopped")
