import cv2 as cv
video = cv.VideoCapture(0);


if (video.isOpened() == False):
    print("Error with opening the file")

video.set(cv.CAP_PROP_FRAME_WIDTH, 640)
video.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
video.set(cv.CAP_PROP_FPS, 30)

count = 0

while True:
    ret, frame = video.read()
    if ret == True:
        cv.imshow("hi", frame)
        k = cv.waitKey(1)
        if k % 256 == 27:
            print("Camera is closing")
            break
        elif k % 256 == 32:
            folder = "calib_img"
            print("A picture has been taken")
            filename = f'{folder}/calibrate{count}.jpg'
            cv.imwrite(filename, frame)
            count += 1
    else:
        break

video.release()
cv.destroyAllWindows()