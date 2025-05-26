import cv2 as cv
import numpy as np
import os

# Load calibration data
data = np.load('camera_calibration.npz')
mtx = data['camera_matrix']
dist = data['dist_coeffs']
newcameramtx = data['new_camera_matrix']
roi = data['roi']