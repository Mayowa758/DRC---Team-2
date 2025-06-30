#!/usr/bin/python
#--------------------------------------
#    ___  ___  _ ____
#   / _ \/ _ \(_) __/__  __ __
#  / , _/ ___/ /\ \/ _ \/ // /
# /_/|_/_/  /_/___/ .__/\_, /
#                /_/   /___/
#
#           servo1.py
#  Basic example script to control a servo
#  connected to GPIO17 using the Gpiozero library.
#
# Use CTRL-C to break out of While loop.
#
# Author : Matt Hawkins
# Date   : 01/01/2018
#
# https://www.raspberrypi-spy.co.uk/tag/servo/
#
#--------------------------------------
from gpiozero import AngularServo
from time import sleep

myGPIO=18


# myServo = Servo(myGPIO)
servo = AngularServo(
  pin=myGPIO,
  min_angle = -50,
  max_angle = 50,
  min_pulse_width= 0.0009,
  max_pulse_width = 0.0021
) 

servo.angle = 0

print("Using GPIO17")
print("Using Gpiozero defaults for the servo class")

def move_servo(start, end, step=1, delay = 0.02):
  if start < end: 
    angle_range = range(start, end + 1, step)
  else:
    angle_range = range(start, end-1, -step)
  for angle in angle_range:
      servo.angle = angle
      sleep(delay)

while True:
  move_servo(0, -30)
  print("Set to min position")
  sleep(1)

  move_servo(-30, 0)
  print("Set to middle position")
  sleep(1)

  move_servo(0, 30)
  print("Set to max position")
  sleep(1)

  move_servo(30, 0)
  print("Set to middle position")
  sleep(1)