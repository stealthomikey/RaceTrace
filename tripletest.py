import numpy as np
import pandas as pd
import math

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

# Define transition segments and corner phases
transition_segments = set()
corner_entry_segments = set()
corner_exit_segments = set()
transition_length = 4  # Number of segments to mark as transition in the middle

for section in corner_sections:
    if len(section) >= transition_length:
        # Calculate the middle index of the corner section
        mid_idx = len(section) // 2
        # Define the range of segments to mark as transition, centered around the middle
        start_idx = section[mid_idx - transition_length // 2]
        end_idx = section[min(len(section) - 1, mid_idx + transition_length // 2)]
        
        # Mark transition segments (apex)
        for idx in range(start_idx, end_idx + 1):
            if idx in section:
                transition_segments.add(idx)
        
        # Mark corner entry (before transition)
        for idx in section:
            if idx < start_idx:
                corner_entry_segments.add(idx)
        
        # Mark corner exit (after transition) - these should accelerate
        for idx in section:
            if idx > end_idx:
                corner_exit_segments.add(idx)

# Calculate distances between consecutive points
distances = []
for i in range(len(x) - 1):
    dist = math.sqrt((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2)
    distances.append(dist)

# Define speed profiles and acceleration limits (m/s and m/s²)
STRAIGHT_SPEED = 60.0    # High speed on straights
CORNER_SPEED = 25.0      # Moderate speed in corners (braking zone)
TRANSITION_SPEED = 15.0  # Slowest speed at apex/transitions

MAX_ACCELERATION = 8.0   # Maximum acceleration (m/s²)
MAX_DECELERATION = -12.0 # Maximum deceleration (m/s²) - negative value

# Determine target speeds for each segment
target_speeds = []
for i in range(len(distances)):
    # Determine segment type
    nearest_angle_idx = min(range(len(angle_indices)), key=lambda j: abs(angle_indices[j] - i))
    nearest_angle = angles[nearest_angle_idx]
    
    is_transition = i in transition_segments
    is_corner_entry = i in corner_entry_segments
    is_corner_exit = i in corner_exit_segments
    is_corner = nearest_angle > angle_threshold and not is_transition and not is_corner_exit
    
    # Assign target speed based on segment type
    if is_transition:
        target_speeds.append(TRANSITION_SPEED)
    elif is_corner_entry:
        target_speeds.append(CORNER_SPEED)
    elif is_corner_exit:
        # Corner exit should target straight speed to encourage acceleration
        target_speeds.append(STRAIGHT_SPEED)
    elif is_corner:
        target_speeds.append(CORNER_SPEED)
    else:
        target_speeds.append(STRAIGHT_SPEED)

# Calculate actual speeds with smooth acceleration/deceleration
actual_speeds = [STRAIGHT_SPEED]  # Start at straight speed
current_speed = STRAIGHT_SPEED

for i in range(len(distances)):
    target_speed = target_speeds[i]
    distance = distances[i]
    
    # Calculate required acceleration to reach target speed
    if target_speed > current_speed:
        # Need to accelerate
        max_possible_accel = MAX_ACCELERATION
        # Calculate speed change possible over this distance
        # Using: v² = u² + 2as, so v = sqrt(u² + 2as)
        max_speed_change = math.sqrt(max(0, current_speed**2 + 2 * max_possible_accel * distance)) - current_speed
        speed_change = min(max_speed_change, target_speed - current_speed)
        new_speed = current_speed + speed_change
    elif target_speed < current_speed:
        # Need to decelerate
        max_possible_decel = MAX_DECELERATION
        # Calculate speed change possible over this distance
        # Using: v² = u² + 2as, where a is negative for deceleration
        max_speed_change = current_speed - math.sqrt(max(0, current_speed**2 + 2 * max_possible_decel * distance))
        speed_change = min(max_speed_change, current_speed - target_speed)
        new_speed = current_speed - speed_change
    else:
        # Speed is already at target
        new_speed = current_speed
    
    # Ensure we don't go below minimum safe speed
    new_speed = max(new_speed, 5.0)
    
    actual_speeds.append(new_speed)
    current_speed = new_speed

# Calculate time for each segment using actual speeds
times = [0.0]  # Start at time 0
current_time = 0.0

for i in range(len(distances)):
    # Use average of start and end speeds for this segment
    start_speed = actual_speeds[i]
    end_speed = actual_speeds[i + 1]
    avg_speed = (start_speed + end_speed) / 2
    
    # Calculate time for this segment
    segment_time = distances[i] / avg_speed
    current_time += segment_time
    times.append(current_time)
    
    # Determine segment type for logging
    nearest_angle_idx = min(range(len(angle_indices)), key=lambda j: abs(angle_indices[j] - i))
    nearest_angle = angles[nearest_angle_idx]
    
    is_transition = i in transition_segments
    is_corner_entry = i in corner_entry_segments
    is_corner_exit = i in corner_exit_segments
    is_corner = nearest_angle > angle_threshold and not is_transition and not is_corner_exit
    
    if is_transition:
        segment_type = "Transition (Apex)"
    elif is_corner_entry:
        segment_type = "Corner Entry"
    elif is_corner_exit:
        segment_type = "Corner Exit"
    elif is_corner:
        segment_type = "Corner"
    else:
        segment_type = "Straight"
    
    # Calculate acceleration for this segment
    if segment_time > 0:
        acceleration = (end_speed - start_speed) / segment_time
    else:
        acceleration = 0
    
    print(f"Segment {i}: {segment_type}, Distance: {distances[i]:.3f}m, "
          f"Speed: {start_speed:.1f}->{end_speed:.1f}m/s, "
          f"Accel: {acceleration:.2f}m/s², Time: {segment_time:.3f}s")

# Create the JavaScript output
js_content = "const carACoordinates = [\n"

for i in range(len(x)):
    js_content += f"    {{ x: {x[i]:.3f}, y: {y[i]:.3f}, time: {times[i]:.3f} }}"
    if i < len(x) - 1:
        js_content += ","
    js_content += "\n"

js_content += "];\n\n"
js_content += "export { carACoordinates as kartCoordinates, carACoordinates as kartLineCoordinates };"

# Write to file
with open('racing_line_with_time.js', 'w') as f:
    f.write(js_content)

print(f"\nGenerated racing line with {len(x)} points")
print(f"Total lap time: {current_time:.3f} seconds")
print(f"Output saved to 'racing_line_with_time.js'")

# Print some statistics
total_distance = sum(distances)
average_speed = total_distance / current_time
print(f"Total distance: {total_distance:.1f}m")
print(f"Average speed: {average_speed:.1f}m/s ({average_speed * 3.6:.1f}km/h)")

# Calculate acceleration statistics
accelerations = []
for i in range(len(distances)):
    if times[i+1] - times[i] > 0:
        accel = (actual_speeds[i+1] - actual_speeds[i]) / (times[i+1] - times[i])
        accelerations.append(accel)

if accelerations:
    max_accel = max(accelerations)
    max_decel = min(accelerations)
    print(f"Maximum acceleration: {max_accel:.2f}m/s²")
    print(f"Maximum deceleration: {max_decel:.2f}m/s²")

# Count segment types
straight_count = sum(1 for i in range(len(distances)) if i not in corner_indices and i not in transition_segments)
corner_entry_count = len(corner_entry_segments)
corner_exit_count = len(corner_exit_segments)
transition_count = len(transition_segments)
other_corner_count = sum(1 for i in range(len(distances)) if i in corner_indices and i not in transition_segments and i not in corner_entry_segments and i not in corner_exit_segments)

print(f"Segment breakdown:")
print(f"  Straights: {straight_count} segments")
print(f"  Corner Entry: {corner_entry_count} segments")
print(f"  Transitions (Apex): {transition_count} segments")
print(f"  Corner Exit: {corner_exit_count} segments") 
print(f"  Other Corners: {other_corner_count} segments")
