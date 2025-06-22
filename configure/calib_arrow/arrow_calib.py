import cv2 as cv
import numpy as np
import sys
import os

# Get the path to the parent directory (configure/)
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)

from undistort_data import *  # This will now work!


video = cv.VideoCapture(0)
left_counter = 0
right_counter = 0
window_width = 640
window_height = 480


video.set(cv.CAP_PROP_FRAME_WIDTH, window_width)
video.set(cv.CAP_PROP_FRAME_HEIGHT, window_height)
video.set(cv.CAP_PROP_FPS, 30)

def get_black_mask(hsv):
    lower = np.array([0, 0, 0])
    upper = np.array([180, 255, 50])  # adjust upper value if needed
    return cv.inRange(hsv, lower, upper)


ret, frame = video.read()
if not ret or frame is None:
    raise RuntimeError("Failed to read frame from camera")
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (window_width, window_height),
                                                0, (window_width, window_height))
h, w = frame.shape[:2]
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), cv.CV_16SC2)

while True:

    _, img = video.read()
    if not _ or img is None:
        raise RuntimeError("Failed to read frame from camera")
    img = cv.remap(img, mapx, mapy, interpolation=cv.INTER_LINEAR)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = get_black_mask(hsv)
    cv.imshow("thing", mask)
    key = cv.waitKey(1) & 0xFF

    # If l is pressed save as a left arrow
    if (key == ord('l')):
        folder = "left_arrow"
        path = f"{folder}/left_arrow{left_counter}.jpg"
        cv.imwrite(path, img)
        print("picture of left_arrow was taken")
        left_counter += 1

    # If r is pressed save as a right arrow
    elif (key == ord('r')):
        folder = "right_arrow"
        path = f"{folder}/right_arrow{right_counter}.jpg"
        cv.imwrite(path, img)
        print("picture of right arrow was taken")
        right_counter += 1
    elif (key == ord('q')):
        break

    cv.imshow("Camera Frame", img)

video.release()
cv.destroyAllWindows()