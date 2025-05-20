from colour_detect import *
from cv2 import cv

new_mask = road_mask(mask_blue, mask_yellow)
video

while True:

    cv.imshow("mask", master_mask)
    cv.imshow("webcam", img)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv.destroyAllWindows()
