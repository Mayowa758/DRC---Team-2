from gpiozero import Servo
from time import sleep
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()

servo = Servo(12, pin_factory=factory)
servo.mid()