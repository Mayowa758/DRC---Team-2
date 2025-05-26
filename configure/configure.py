import numpy as np
import cv2 as cv
import glob

# Termination criteria for corner refinement
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points for a 7x6 chessboard (7 cols, 6 rows)
objp = np.zeros((6 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

images = glob.glob("calib_img/*.jpg")
gray_shape = None

for fname in images:
    img = cv.imread(fname)
    if img is None:
        print(f"Warning: Could not read {fname}")
        continue

    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv.findChessboardCorners(gray, (7, 6), None)

    if ret:
        objpoints.append(objp)
        gray_shape = gray.shape[::-1]  # Set once when a good image is found

        # Refine corners
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Optionally visualize
        cv.drawChessboardCorners(img, (7, 6), corners2, ret)
        cv.imshow('Chessboard', img)
        cv.waitKey(500)

cv.destroyAllWindows()

# Safety check
if len(objpoints) == 0 or gray_shape is None:
    raise ValueError("No valid chessboard corners were detected.")

print(f"Calibration images used: {len(objpoints)}")

# Camera calibration
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray_shape, None, None)

print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)

# Load an image to undistort
img = cv.imread('calib_img/calibrate0.jpg')
if img is None:
    raise FileNotFoundError("Could not read 'calibrate0.jpg'.")

h, w = img.shape[:2]
newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# Undistort
mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)
dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

# Optionally crop using ROI
x, y, w_roi, h_roi = roi
if w_roi > 0 and h_roi > 0:
    dst = dst[y:y+h_roi, x:x+w_roi]

cv.imwrite('calibresult.png', dst)
cv.imshow('Original', img)
cv.imshow('Undistorted', dst)
cv.waitKey(0)
cv.destroyAllWindows()

# Reprojection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
    mean_error += error

print("Total reprojection error:", mean_error / len(objpoints))

np.savez('camera_calibration.npz', 
         camera_matrix=mtx, 
         dist_coeffs=dist, 
         new_camera_matrix=newcameramtx, 
         roi=roi)