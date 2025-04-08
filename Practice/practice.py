# from gpiozero import MotionSensor, LED
# from signal import pause


# pir = MotionSensor(4)
# led = LED(16)

# pir.when_motion = led.on
# pir.when_no_motion = led.off

# pause()


import numpy as np
import cv2 as cv

vid = cv.VideoCapture(0) # need to set this video capture to the camera

while True:
    ret, frame = vid.read()
    width = int(vid.get(3))
    height = int(vid.get(4))

    cv.imshow('Hello', frame)

    if cv.waitKey(1) == ord('d'):
        break

vid.release()
cv.destroyAllWindows()