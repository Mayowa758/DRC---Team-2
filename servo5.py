# Im gonna add some test code here
import pigpio
from time import sleep

# connect to the 
pi = pigpio.pi()

# loop forever

pi.set_servo_pulsewidth(18, 0)    # off
sleep(1)
pi.set_servo_pulsewidth(18, 1000) # position anti-clockwise
sleep(1)
pi.set_servo_pulsewidth(18, 1500) # middle
sleep(1)
pi.set_servo_pulsewidth(18, 2000) # position clockwise
sleep(1)

pi.set_servo_pulsewidth(18, 1500)
pi.set_servo_pulsewidth(18, 0)
pi.stop()