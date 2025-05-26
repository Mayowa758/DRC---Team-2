
# Plan
# 1. Find centreline using row-wise midpoints between the boundaries
# 2. Smooth the centreline using np.polyfit() or spline interpolation
# 3. Follow the path using:
#   - Pure Pursuit for steering
#   - Curvature-based speed control using a formula like:
#       v = min(v_max, 1/(k . curvature))



# 1. Find centreline of track

# Use OpenCV to detect the two tape boundaries (they’ll be the largest contours).
contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Sort contours by area (largest two are probably your blue and yellow tapes):
contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]


# Approximate the Centreline
# Find the left and right boundary pixels and take their midpoint as the centre of that row
height, width = binary.shape
centerline = []

for y in range(0, height, 10):  # every 10 pixels vertically
    row = binary[y, :]
    left = np.argmax(row > 0)
    right = width - np.argmax(np.flip(row) > 0)
    
    if left < right:  # Valid track section
        mid = (left + right) // 2
        centerline.append((mid, y))


# Fitting the trail to a curve
# This is for the steering logic
points = np.array(centerline)
fit = np.polyfit(points[:, 1], points[:, 0], deg=2)  # x = f(y)


# Debugging
for x, y in centerline:
    cv2.circle(image, (x, y), 2, (0, 255, 0), -1)

cv2.imshow("Trail Center", image)





# Python code to compute curvature and speed

import numpy as np
import matplotlib.pyplot as plt

# Example list of centreline points (x, y)
points = np.array([
    [0, 0],
    [1, 0.2],
    [2, 0.8],
    [3, 1.8],
    [4, 3.2],
    [5, 5],
    [6, 7.2]
])

# Extract x and y arrays
x = points[:, 0]
y = points[:, 1]

# Compute first and second derivatives using central differences
dx = np.gradient(x)
dy = np.gradient(y)
ddx = np.gradient(dx)
ddy = np.gradient(dy)

# Compute curvature at each point using parametric formula
# kappa = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
numerator = np.abs(dx * ddy - dy * ddx)
denominator = (dx**2 + dy**2)**1.5
curvature = numerator / denominator

# Handle any divide-by-zero issues
curvature = np.nan_to_num(curvature)

# Calculate speed: v = min(v_max, 1 / (k * curvature))
v_max = 2.0  # maximum speed
k = 2.0      # tuning parameter
speed = np.minimum(v_max, 1 / (k * curvature + 1e-6))  # add epsilon to avoid divide by zero

# Plotting (optional, for understanding)
plt.figure(figsize=(8, 6))
plt.plot(x, y, label='Centreline', marker='o')
for i in range(len(x)):
    plt.text(x[i], y[i], f"v={speed[i]:.2f}", fontsize=9)
plt.title("Centreline and Computed Speeds")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()





# Combined Pure Pursuit and Speed Control code
import numpy as np
import matplotlib.pyplot as plt

def compute_curvature_and_speed(points, v_max=2.0, k=2.0):
    x = points[:, 0]
    y = points[:, 1]
    dx = np.gradient(x)
    dy = np.gradient(y)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    numerator = np.abs(dx * ddy - dy * ddx)
    denominator = (dx**2 + dy**2)**1.5
    curvature = numerator / denominator
    curvature = np.nan_to_num(curvature)
    speed = np.minimum(v_max, 1 / (k * curvature + 1e-6))
    return curvature, speed

def find_lookahead_point(car_pos, path_points, lookahead_distance):
    for i in range(len(path_points) - 1):
        start = path_points[i]
        end = path_points[i + 1]
        d = end - start
        f = start - car_pos
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - lookahead_distance**2
        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            continue
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        for t in [t1, t2]:
            if 0 <= t <= 1:
                return start + t * d
    return path_points[-1]  # fallback

def pure_pursuit_steering(car_pos, car_heading, lookahead_point, wheelbase=0.3):
    dx = lookahead_point[0] - car_pos[0]
    dy = lookahead_point[1] - car_pos[1]
    transformed_x = np.cos(-car_heading) * dx - np.sin(-car_heading) * dy
    transformed_y = np.sin(-car_heading) * dx + np.cos(-car_heading) * dy
    if transformed_x == 0:
        return 0.0  # avoid divide by zero
    curvature = 2 * transformed_y / (lookahead_distance**2)
    steering_angle = np.arctan(wheelbase * curvature)
    return steering_angle

# === Example Usage ===

# Simulated path (replace with your real centreline)
path = np.array([
    [0, 0],
    [1, 0.2],
    [2, 0.8],
    [3, 1.8],
    [4, 3.2],
    [5, 5],
    [6, 7.2]
])

# Step 1: Compute curvature and speeds
curvature, speeds = compute_curvature_and_speed(path)

# Step 2: Current car state
car_position = np.array([1.5, 0.5])     # x, y
car_heading = np.radians(15)           # in radians
lookahead_distance = 1.0               # adjustable

# Step 3: Find lookahead point on the path
lookahead_point = find_lookahead_point(car_position, path, lookahead_distance)

# Step 4: Compute steering angle
steering_angle = pure_pursuit_steering(car_position, car_heading, lookahead_point)

# Step 5: Estimate speed at nearest path point
distances = np.linalg.norm(path - car_position, axis=1)
nearest_index = np.argmin(distances)
car_speed = speeds[nearest_index]

# Final output
print(f"Steering angle: {np.degrees(steering_angle):.2f} degrees")
print(f"Target speed: {car_speed:.2f} m/s")

