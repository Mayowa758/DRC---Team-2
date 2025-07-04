import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt


def make_coordinates(image, line_parameters):
    slope, intercept = line_parameters
    y1 = image.shape[0]
    y2 = int(y1*(3/5))
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])


def average_slope_intercept(image, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        x1, y1, x2, y2 = line.reshape(4)
        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        intercept = parameters[1]
        # lines on left - negative slope
        # lines on right - positive slope

        if slope < 0:
            left_fit.append((slope,intercept))
        else:
            right_fit.append((slope, intercept))

        # print(left_fit)
        # print(right_fit)
        # print(parameters)

    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)

    left_line = make_coordinates(image, left_fit_average)
    right_line = make_coordinates(image, right_fit_average)
    return np.array([left_line, right_line])
        # print(left_fit_average)
        # print(right_fit_average)

def canny(image):
    gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
    blur = cv.GaussianBlur(gray, (5,5), 0)
    canny = cv.Canny(blur, 50, 150)
    return canny

def region_of_interest(image):
    height = image.shape[0] # number of rows (the height)
    polygons = np.array([
        [(200, height), (1100, height), (550, 250)]])
    mask = np.zeros_like(image)
    cv.fillPoly(mask, polygons, 255)

    masked_image = cv.bitwise_and(image, mask)
    return masked_image

def display_lines(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            cv.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10) # BGR

    return line_image


img = cv.imread('Photos/road.jpg');

# if lane_image = img when one changes they both change

lane_image = np.copy(img)



cap = cv.VideoCapture("Photos/road_video.mp4")
while(cap.isOpened()):
    _, frame = cap.read()

    canny_image = canny(frame)

    # print(img)
    cropped_image = region_of_interest(canny_image)
    lines = cv.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    average_lines = average_slope_intercept(frame, lines)

    line_image = display_lines(frame, average_lines)
    combo_image = cv.addWeighted(frame, 0.8, line_image, 1, 1)
    cv.imshow("result", combo_image)
    cv.waitKey(1);