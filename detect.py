import cv2 as cv
import numpy as np

vid = cv.VideoCapture(0) # get photo for camera

while True:
    ret, frame = vid.read()
    # width = int(vid.get(3))
    # height = int(vid.get(4))

    cv.imshow('Video Detection', frame)
    if cv.waitKey(1) == ord('q'):
        break




vid.release() # this release memory

cv.destroyAllWindows()