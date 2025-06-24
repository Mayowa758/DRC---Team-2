import cv2 as cv
import numpy as np
import os

# Load calibration data
base_dir = os.path.dirname(__file__)
calib_path = os.path.join(base_dir, 'camera_calibration.npz')
data = np.load(calib_path)
mtx = data['camera_matrix']
dist = data['dist_coeffs']
newcameramtx = data['new_camera_matrix']
roi = data['roi']