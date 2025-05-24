import cv2 as cv
video = cv.VideoCapture(0);

video.set(cv.CAP_PROP_FRAME_WIDTH, 640)
video.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
video.set(cv.CAP_PROP_FPS, 30)

while True:

    ret, frame = video.read()
    cv.imshow("hi", frame);
    
    k = cv.waitKey(1)

    count = 0
    if k % 256 == 27:
        print("Camera is closing")
        break
    elif k % 256 == 32:
        print("A picture has been taken")
        filename = f'calibrate{count}.jpg'
        cv.imwrite(filename, frame)
        count += 1

video.release()
cv.destroyAllWindows()