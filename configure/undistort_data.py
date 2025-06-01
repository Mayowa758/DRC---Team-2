import cv2 as cv
import numpy as np

# Load calibration data
data = np.load('configure/camera_calibration.npz')
mtx = data['camera_matrix']
dist = data['dist_coeffs']
newcameramtx = data['new_camera_matrix']
roi = data['roi']