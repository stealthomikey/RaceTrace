import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
try:
    df = pd.read_csv('racing_line.csv', comment='#', header=None, names=['x_m', 'y_m'])
    data = list(zip(df['x_m'], df['y_m']))
except FileNotFoundError:
    print("Error: 'racing_line.csv' not found. Please ensure the file is in the working directory.")
    exit(1)
except Exception as e:
    print(f"Error reading CSV file: {e}")
    exit(1)

# Extract x and y coordinates
x, y = zip(*data)

# Function to calculate angle between three points (in degrees)
def calculate_angle(p1, p2, p3):
    v1 = np.array([p1[0] - p2[0], p1[1] - p2[1]])
    v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
    mag1 = np.sqrt(v1.dot(v1))
    mag2 = np.sqrt(v2.dot(v2))
    if mag1 == 0 or mag2 == 0:
        return 0
    cos_angle = v1.dot(v2) / (mag1 * mag2)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.degrees(np.arccos(cos_angle))
    return min(angle, 180 - angle)

# Calculate angles using every nth point to capture broader turns
n = 10  # Use every 10th point to calculate angles
angles = []
angle_indices = []
for i in range(n, len(data) - n, 1):  # Step by 1, but look at points n steps apart
    angle = calculate_angle(data[i-n], data[i], data[i+n])
    angles.append(angle)
    angle_indices.append(i)
    print(f"Angle at point {i}: {angle:.2f} degrees")

# Threshold for considering a segment a corner (in degrees)
angle_threshold = 5  # Lowered to detect small but significant turns

# Identify corner segments
corner_indices = []
for i in range(len(x) - 1):
    nearest_angle_idx = min(range(len(angle_indices)), key=lambda j: abs(angle_indices[j] - i))
    nearest_angle = angles[nearest_angle_idx]
    if nearest_angle > angle_threshold:
        corner_indices.append(i)

# Group consecutive corner segments into corner sections
corner_sections = []
current_section = []
for i in range(len(x) - 1):
    if i in corner_indices:
        current_section.append(i)
    else:
        if current_section:
            corner_sections.append(current_section)
            current_section = []
if current_section:
    corner_sections.append(current_section)

# Define transition segments in the middle of each corner section
transition_segments = set()
transition_length = 4  # Number of segments to mark as transition in the middle
for section in corner_sections:
    if len(section) >= transition_length:
        # Calculate the middle index of the corner section
        mid_idx = len(section) // 2
        # Define the range of segments to mark as transition, centered around the middle
        start_idx = section[mid_idx - transition_length // 2]
        end_idx = section[min(len(section) - 1, mid_idx + transition_length // 2)]
        for idx in range(start_idx, end_idx + 1):
            if idx in section:
                transition_segments.add(idx)

# Plot the circuit
plt.figure(figsize=(10, 8))

# Plot each segment, classifying as corner, transition, or straight
for i in range(len(x) - 1):
    nearest_angle_idx = min(range(len(angle_indices)), key=lambda j: abs(angle_indices[j] - i))
    nearest_angle = angles[nearest_angle_idx]
    
    # Classify as corner if the nearest angle exceeds the threshold and not a transition
    is_corner = nearest_angle > angle_threshold and i not in transition_segments
    is_transition = i in transition_segments
    
    # Assign color based on classification
    if is_corner:
        color = 'red'
        label = 'Corner (red)'
    elif is_transition:
        color = 'purple'
        label = 'Transition (purple)'
    else:
        color = 'blue'
        label = 'Straight (blue)'
    
    print(f"Segment {i} to {i+1}: {label}")
    plt.plot([x[i], x[i+1]], [y[i], y[i+1]], color=color, linewidth=2)

# Set axis limits as specified
plt.xlim(-550, 1500)
plt.ylim(-500, 1250)

# Add labels and title
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Racing Line: Corners (Red), Transitions (Purple), Straights (Blue)')
plt.grid(True)
plt.axis('equal')

# Save the plot
plt.savefig('racing_line_plot.png')
