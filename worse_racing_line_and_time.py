import math
import csv
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.signal import find_peaks
import matplotlib.pyplot as plt

# Read CSV data
points = []
with open('racing_line.csv', 'r') as file:
    reader = csv.reader(file)
    next(reader)  # Skip header
    for row in reader:
        x, y = map(float, row)
        points.append((x, y))

# Convert points to numpy arrays
x = np.array([p[0] for p in points])
y = np.array([p[1] for p in points])

# Calculate cumulative arc length for parameterization
distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
s = np.concatenate(([0], np.cumsum(distances)))

# Smooth the path with fewer points
num_smooth_points = 100
s_smooth = np.linspace(0, s[-1], num_smooth_points)
spline_x = CubicSpline(s, x, bc_type='periodic')  # Periodic for closed circuit
spline_y = CubicSpline(s, y, bc_type='periodic')
x_smooth = spline_x(s_smooth)
y_smooth = spline_y(s_smooth)

# Calculate curvature of the smoothed path
dx = spline_x(s_smooth, 1)
dy = spline_y(s_smooth, 1)
d2x = spline_x(s_smooth, 2)
d2y = spline_y(s_smooth, 2)
curvature = np.abs(dx * d2y - dy * d2x) / np.where((dx**2 + dy**2)**1.5 == 0, 1e-10, (dx**2 + dy**2)**1.5)
curvature = np.where(np.isnan(curvature), 0, curvature)

# Identify corners using peak detection
curvature_threshold = 0.05  # Approximate threshold for corners (~20m radius)
peaks, _ = find_peaks(curvature, height=curvature_threshold, distance=5)
corner_indices = peaks
corner_s = s_smooth[corner_indices]

# Define corner regions (extend around peaks)
corner_ranges = []
window = s_smooth[-1] / num_smooth_points * 3  # Window around peaks (~3 smooth points)
for cs in corner_s:
    corner_ranges.append((cs - window, cs + window))

# Assign speeds to original points
speeds = np.full(len(points), 30.0)  # Default to straight speed
for i, s_val in enumerate(s):
    for start, end in corner_ranges:
        if start <= s_val <= end or (start > end and (s_val >= start or s_val <= end)):  # Handle wraparound
            speeds[i] = 5.0  # Corner speed
            break

# Calculate times
times = [0.0]
current_time = 0.0
for i in range(len(distances)):
    time_increment = distances[i] / speeds[i]
    current_time += time_increment
    times.append(current_time)

# Scale times to 90 seconds
target_time = 90.0
scale_factor = target_time / times[-1]
times = np.array(times) * scale_factor

# Plot the circuit and highlight corners
plt.figure(figsize=(10, 8))
plt.plot(x, y, 'b-', label='Original Path')
plt.plot(x_smooth, y_smooth, 'r--', label='Smoothed Path')
for start, end in corner_ranges:
    mask = (s >= start) & (s <= end) if start <= end else (s >= start) | (s <= end)
    plt.plot(x[mask], y[mask], 'g.', label='Corner' if start == corner_ranges[0][0] else '')
plt.scatter(x_smooth[corner_indices], y_smooth[corner_indices], c='red', marker='x', s=100, label='Corner Peaks')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Racing Line with Detected Corners')
plt.legend()
plt.grid(True)
plt.savefig('racing_line_corners.png')
plt.close()

# Generate JavaScript file
with open('kart_coordinates.js', 'w') as js_file:
    js_file.write('const carACoordinates = [\n')
    for i, ((x_val, y_val), t) in enumerate(zip(points, times)):
        js_file.write(f'\t{{ x: {x_val}, y: {y_val}, time: {t:.3f} }}')
        if i < len(points) - 1:
            js_file.write(',')
        js_file.write('\n')
    js_file.write('];\n')
    js_file.write('export { carACoordinates as kartCoordinates, carACoordinates as kartLineCoordinates };\n')

print("JavaScript file 'kart_coordinates.js' and plot 'racing_line_corners.png' have been generated.")
