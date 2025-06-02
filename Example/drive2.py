# PLAN:
# 1. Find centreline using row-wise midpoints between the boundaries
# 2. Smooth the centreline using np.polyfit() or spline interpolation
# 3. Follow the path using:
#   - Pure Pursuit for steering
#   - Curvature-based speed control using a formula like:
#       v = min(v_max, 1/(k . curvature))



# 1. ######################### FIND CENTRELINE OF TRACK ##########################

# Approximate the Centreline
# Find the left and right boundary pixels and take their midpoint as the centre of that row
import cv2
import numpy as np

# Load the image (grayscale)
image = cv2.imread('track.png', cv2.IMREAD_GRAYSCALE)

# Threshold the image to get binary mask (track is white)
_, binary = cv2.threshold(image, thresh=127, maxval=255, type=cv2.THRESH_BINARY)

# Optional: Convert to boolean or 0/1 if you prefer
binary = (binary > 0).astype(np.uint8)

## You need an image to define binary
height, width = binary.shape
centerline = []

for y in range(0, height, 10):  # every 10 pixels vertically
    row = binary[y, :]
    left = np.argmax(row > 0)
    right = width - np.argmax(np.flip(row) > 0)
    
    if left < right:  # Valid track section
        mid = (left + right) // 2
        centerline.append((mid, y))



# 2. ############################## SMOOTHING THE CENTRELINE ##############################

# Fitting the trail to a curve
# This is for the steering logic
points = np.array(centerline)
fit = np.polyfit(points[:, 1], points[:, 0], deg=2)  # x = f(y)



# 3. ############################### DETERMINING ANGLE AND SPEED FROM SMOOTHED CENTRELINE ##################################3

# Pure Pursuit and curvature-based speed control
def compute_curvature_and_speed(points, v_max=2.0, k=2.0):
    # Collates Data
    x = points[:, 0]
    y = points[:, 1]

    # Calculates the gradient at that point etc...
    dx = np.gradient(x)
    dy = np.gradient(y)

    # Concavity
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    # Finding the curvature of the path
    numerator = np.abs(dx * ddy - dy * ddx)
    denominator = (dx*2 + dy*2)*1.5
    curvature = numerator / denominator
    curvature = np.nan_to_num(curvature)

    # Finding the speed
    speed = np.minimum(v_max, 1 / (k * curvature + 1e-6)) # uses curvature formula
    return curvature, speed

def find_lookahead_point(car_pos, path_points, lookahead_distance):


    for i in range(len(path_points) - 1):

        # It grabs the first and last points
        start = path_points[i]
        end = path_points[i + 1]

        #Finding the distance
        d = end - start

        # Distance between car and supposed start point
        f = start - car_pos

        # Need to research this.
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - lookahead_distance**2
        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            continue
        discriminant = np.sqrt(discriminant)

        ## Finds the intersection points
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        for t in [t1, t2]:
            if 0 <= t <= 1:
                return start + t * d
    return path_points[-1]  # fallback

def pure_pursuit_steering(car_pos, car_heading, lookahead_point, wheelbase=0.3):
    dx = lookahead_point[0] - car_pos[0]
    dy = lookahead_point[1] - car_pos[1]

    # Transform the whole image by car_heading degrees
    transformed_x = np.cos(-car_heading) * dx - np.sin(-car_heading) * dy
    transformed_y = np.sin(-car_heading) * dx + np.cos(-car_heading) * dy
    if transformed_x == 0:
        return 0.0  # avoid divide by zero
    curvature = 2 * transformed_y / (lookahead_distance * 2)
    steering_angle = np.arctan(wheelbase * curvature)
    return steering_angle