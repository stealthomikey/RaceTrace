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

# Define transition segments and corner phases with earlier braking zones
transition_segments = set()
corner_entry_segments = set()
corner_exit_segments = set()
braking_zone_segments = set()  # New: early braking zone before corner entry
acceleration_zone_segments = set()  # New: early acceleration zone after apex

transition_length = 4  # Number of segments to mark as transition in the middle
braking_extension = 15  # Additional segments before corner entry for early braking (increased further)
acceleration_extension = 8  # Start acceleration this many segments after apex (increased)

# Process ALL corner sections, regardless of length
for section in corner_sections:
    section_length = len(section)
    
    if section_length >= transition_length:
        # For longer corners, use the original logic
        mid_idx = len(section) // 2
        start_idx = section[mid_idx - transition_length // 2]
        end_idx = section[min(len(section) - 1, mid_idx + transition_length // 2)]
    else:
        # For shorter corners, use the entire section as transition
        start_idx = section[0]
        end_idx = section[-1]
    
    # Mark transition segments (apex)
    for idx in range(start_idx, end_idx + 1):
        if idx in section:
            transition_segments.add(idx)
    
    # Mark corner entry (before transition, but after braking zone)
    entry_start = max(0, section[0])
    for idx in section:
        if idx < start_idx and idx >= entry_start:
            corner_entry_segments.add(idx)
    
    # Mark early braking zone (ALWAYS create braking zone for every corner)
    braking_start = max(0, section[0] - braking_extension)
    braking_end = section[0]
    for idx in range(braking_start, braking_end):
        if idx >= 0 and idx < len(x) - 1:
            braking_zone_segments.add(idx)
    
    # Mark corner exit (immediately after transition)
    if section_length >= transition_length:
        immediate_exit_end = min(len(section) - 1, mid_idx + transition_length // 2 + 3)
        for idx in section:
            if idx > end_idx and idx <= (section[immediate_exit_end] if immediate_exit_end < len(section) else section[-1]):
                corner_exit_segments.add(idx)
    else:
        # For short corners, mark a few segments after as exit
        exit_length = min(3, section_length)
        for i in range(exit_length):
            exit_idx = section[-1] + i + 1
            if exit_idx < len(x) - 1:
                corner_exit_segments.add(exit_idx)
    
    # Mark early acceleration zone (starts right after apex for ALL corners)
    accel_start = end_idx + 1  # Start right after apex
    accel_end = min(len(x) - 1, section[-1] + acceleration_extension)
    for idx in range(accel_start, accel_end + 1):
        if idx < len(x) - 1:
            acceleration_zone_segments.add(idx)

# Also add braking zones for isolated corner segments that might not be in sections
for i in range(len(x) - 1):
    nearest_angle_idx = min(range(len(angle_indices)), key=lambda j: abs(angle_indices[j] - i))
    nearest_angle = angles[nearest_angle_idx]
    
    # If this is a corner segment but not already in a braking zone, add braking ahead of it
    if nearest_angle > angle_threshold and i not in braking_zone_segments:
        # Look ahead to see if there's already a braking zone
        has_braking_ahead = any(j in braking_zone_segments for j in range(max(0, i-braking_extension), i))
        
        if not has_braking_ahead:
            # Add braking zone before this corner
            for j in range(max(0, i - braking_extension), i):
                if j >= 0 and j < len(x) - 1:
                    braking_zone_segments.add(j)

# Calculate distances between consecutive points
distances = []
for i in range(len(x) - 1):
    dist = math.sqrt((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2)
    distances.append(dist)

# Define speed profiles and acceleration limits (m/s and m/s²)
STRAIGHT_SPEED = 100.0        # Very high speed on straights
BRAKING_ZONE_SPEED = 45.0    # Dramatic speed reduction in braking zone
CORNER_SPEED = 30.0          # Much slower in corners  
TRANSITION_SPEED = 27.0      # Very slow at apex/transitions
ACCELERATION_ZONE_SPEED = 65.0  # Medium speed during early acceleration

MAX_ACCELERATION = 15.0      # Very aggressive acceleration (m/s²)
MAX_DECELERATION = -25.0     # Very aggressive deceleration (m/s²) - negative value

# Determine target speeds for each segment
target_speeds = []
for i in range(len(distances)):
    # Determine segment type
    nearest_angle_idx = min(range(len(angle_indices)), key=lambda j: abs(angle_indices[j] - i))
    nearest_angle = angles[nearest_angle_idx]
    
    is_transition = i in transition_segments
    is_corner_entry = i in corner_entry_segments
    is_corner_exit = i in corner_exit_segments
    is_braking_zone = i in braking_zone_segments
    is_acceleration_zone = i in acceleration_zone_segments
    is_corner = nearest_angle > angle_threshold and not is_transition and not is_corner_exit and not is_braking_zone
    
    # Assign target speed based on segment type (prioritize specific zones)
    if is_transition:
        target_speeds.append(TRANSITION_SPEED)
    elif is_braking_zone:
        target_speeds.append(BRAKING_ZONE_SPEED)
    elif is_corner_entry:
        target_speeds.append(CORNER_SPEED)
    elif is_acceleration_zone:
        target_speeds.append(ACCELERATION_ZONE_SPEED)
    elif is_corner_exit:
        target_speeds.append(STRAIGHT_SPEED)  # Target straight speed for acceleration
    elif is_corner:
        target_speeds.append(CORNER_SPEED)
    else:
        target_speeds.append(STRAIGHT_SPEED)

# Calculate actual speeds with sharp acceleration/deceleration transitions  
actual_speeds = [STRAIGHT_SPEED]  # Start at straight speed
current_speed = STRAIGHT_SPEED

for i in range(len(distances)):
    target_speed = target_speeds[i]
    distance = distances[i]
    
    # Make speed changes much more aggressive and immediate
    speed_difference = target_speed - current_speed
    
    if abs(speed_difference) < 2.0:
        # If close to target, just set to target
        new_speed = target_speed
    elif target_speed > current_speed:
        # Need to accelerate - use maximum acceleration
        max_possible_accel = MAX_ACCELERATION
        # Calculate speed change possible over this distance
        max_speed_change = math.sqrt(max(0, current_speed**2 + 2 * max_possible_accel * distance)) - current_speed
        # Take the full acceleration or reach target, whichever is smaller
        speed_change = min(max_speed_change, target_speed - current_speed)
        new_speed = current_speed + speed_change
        
        # If we can reach target speed easily, do it
        if speed_change > (target_speed - current_speed) * 0.8:
            new_speed = target_speed
            
    elif target_speed < current_speed:
        # Need to decelerate - use maximum deceleration
        max_possible_decel = MAX_DECELERATION
        # Calculate speed change possible over this distance  
        max_speed_change = current_speed - math.sqrt(max(0, current_speed**2 + 2 * max_possible_decel * distance))
        # Take the full deceleration or reach target, whichever is smaller
        speed_change = min(max_speed_change, current_speed - target_speed)
        new_speed = current_speed - speed_change
        
        # If we can reach target speed easily, do it
        if speed_change > (current_speed - target_speed) * 0.8:
            new_speed = target_speed
    else:
        new_speed = current_speed
    
    # Ensure we don't go below minimum safe speed
    new_speed = max(new_speed, 8.0)
    
    actual_speeds.append(new_speed)
    current_speed = new_speed

# Calculate time for each segment using actual speeds
times = [0.0]  # Start at time 0
current_time = 0.0

# Also calculate for Car B (offset car)
times_car_b = [0.0]
current_time_b = 0.0
actual_speeds_b = [STRAIGHT_SPEED]  # Car B starts at same speed
current_speed_b = STRAIGHT_SPEED

for i in range(len(distances)):
    # Car A (original calculations)
    start_speed = actual_speeds[i]
    end_speed = actual_speeds[i + 1]
    avg_speed = (start_speed + end_speed) / 2
    
    segment_time = distances[i] / avg_speed
    current_time += segment_time
    times.append(current_time)
    
    # Car B calculations (slightly slower but similar speeds)
    target_speed_b = target_speeds[i] * 1.011  # 2% slower speeds for more realistic racing
    distance_b = distances[i]  # Same distance for racing line
    
    # Calculate Car B speed with same acceleration logic
    if target_speed_b > current_speed_b:
        max_speed_change_b = math.sqrt(max(0, current_speed_b**2 + 2 * MAX_ACCELERATION * distance_b)) - current_speed_b
        speed_change_b = min(max_speed_change_b, target_speed_b - current_speed_b)
        new_speed_b = current_speed_b + speed_change_b
    elif target_speed_b < current_speed_b:
        max_speed_change_b = current_speed_b - math.sqrt(max(0, current_speed_b**2 + 2 * MAX_DECELERATION * distance_b))
        speed_change_b = min(max_speed_change_b, current_speed_b - target_speed_b)
        new_speed_b = current_speed_b - speed_change_b
    else:
        new_speed_b = current_speed_b
    
    new_speed_b = max(new_speed_b, 5.0)
    actual_speeds_b.append(new_speed_b)
    
    # Calculate time for Car B
    start_speed_b = current_speed_b
    end_speed_b = new_speed_b
    avg_speed_b = (start_speed_b + end_speed_b) / 2
    segment_time_b = distance_b / avg_speed_b
    current_time_b += segment_time_b
    times_car_b.append(current_time_b)
    current_speed_b = new_speed_b

for i in range(len(distances)):
    # Determine segment type for logging
    nearest_angle_idx = min(range(len(angle_indices)), key=lambda j: abs(angle_indices[j] - i))
    nearest_angle = angles[nearest_angle_idx]
    
    is_transition = i in transition_segments
    is_corner_entry = i in corner_entry_segments
    is_corner_exit = i in corner_exit_segments
    is_braking_zone = i in braking_zone_segments
    is_acceleration_zone = i in acceleration_zone_segments
    is_corner = nearest_angle > angle_threshold and not is_transition and not is_corner_exit and not is_braking_zone
    
    if is_transition:
        segment_type = "Transition (Apex)"
    elif is_braking_zone:
        segment_type = "Braking Zone"
    elif is_corner_entry:
        segment_type = "Corner Entry"
    elif is_acceleration_zone:
        segment_type = "Acceleration Zone"
    elif is_corner_exit:
        segment_type = "Corner Exit"
    elif is_corner:
        segment_type = "Corner"
    else:
        segment_type = "Straight"
    
    # Calculate acceleration for Car A
    if (i + 1) < len(times) and times[i+1] - times[i] > 0:
        acceleration = (actual_speeds[i+1] - actual_speeds[i]) / (times[i+1] - times[i])
    else:
        acceleration = 0
    
    # print(f"Segment {i}: {segment_type}, Distance: {distances[i]:.3f}m, "
    #           f"Car A Speed: {actual_speeds[i]:.1f}->{actual_speeds[i+1]:.1f}m/s, "
    #           f"Car B Speed: {actual_speeds_b[i]:.1f}->{actual_speeds_b[i+1]:.1f}m/s, "
    #           f"Accel: {acceleration:.2f}m/s², Time A: {times[i+1] - times[i]:.3f}s, Time B: {times_car_b[i+1] - times_car_b[i]:.3f}s")

# Generate Car B coordinates (wider racing line + offset on straights)
# This will be the RAW, potentially unsmooth path
raw_x_b = [0.0] * len(x)
raw_y_b = [0.0] * len(y)

# --- REVISED PARAMETERS AND LOGIC FOR SMOOTHER TRANSITION AND CORRECT CORNERING ---
SMOOTH_OFFSET_TRANSITION_POINTS = 20 # Points for the lateral offset to ramp up/down
SMOOTH_DIRECTION_WINDOW = 5 # Window for averaging turn direction (cross product)

STRAIGHT_LATERAL_OFFSET = 0.0 # Lateral offset for Car B on straights
CORNER_TARGET_LATERAL_OFFSET = 2.5 # Maximum lateral offset for Car B in corners (outward)

# Store the start, end, and geometric center for each corner section
# This is used to define the "outward" direction for the target path
corner_details = []
for section in corner_sections:
    if section:
        section_start_idx = section[0]
        section_end_idx = section[-1]
        
        # Calculate the geometric center of this corner for determining "outward" direction
        corner_x_coords = [x[j] for j in section]
        corner_y_coords = [y[j] for j in section]
        center_x = sum(corner_x_coords) / len(corner_x_coords)
        center_y = sum(corner_y_coords) / len(corner_y_coords)
        
        corner_details.append({
            'start_idx': section_start_idx,
            'end_idx': section_end_idx,
            'center_x': center_x,
            'center_y': center_y
        })

# --- Helper to get the correct "outward" normal for Car B's path ---
def get_outward_normal(current_idx, path_x, path_y, corner_sections_info):
    # Calculate stable direction vector (tangent) for Car A's line
    look_behind_dir = max(0, current_idx - 10) 
    look_ahead_dir = min(len(path_x) - 1, current_idx + 10) 
    
    direction_x = 0
    direction_y = 0
    
    if look_ahead_dir > look_behind_dir: 
        direction_x = path_x[look_ahead_dir] - path_x[look_behind_dir]
        direction_y = path_y[look_ahead_dir] - path_y[look_behind_dir]
    elif current_idx + 1 < len(path_x): # Fallback for near start
        direction_x = path_x[current_idx+1] - path_x[current_idx]
        direction_y = path_y[current_idx+1] - path_y[current_idx]
    elif current_idx > 0: # Fallback for near end
        direction_x = path_x[current_idx] - path_x[current_idx-1]
        direction_y = path_y[current_idx] - path_y[current_idx-1]
            
    direction_length = math.sqrt(direction_x**2 + direction_y**2)
    
    perp_x = 0
    perp_y = 0
    
    if direction_length > 0: 
        direction_x /= direction_length
        direction_y /= direction_length
        
        # Perpendicular vector to the racing line (points "left" if direction is "forward")
        perp_x = -direction_y
        perp_y = direction_x

    # Determine which way is "outward" from the corner
    # This logic goes back to using the corner center for robust outward direction
    # This was the logic that achieved "wider at corners" before smoothing issues.
    offset_direction_multiplier = 1 # Default to perp_x, perp_y as "outward"

    active_corner_for_direction = None
    for c_info in corner_sections_info:
        # Check if the current point is inside the primary influence zone for direction
        if current_idx >= c_info['start_idx'] and current_idx <= c_info['end_idx']:
            active_corner_for_direction = c_info
            break
        # Also consider points slightly before/after a corner if it's the only one
        elif current_idx >= (c_info['start_idx'] - SMOOTH_OFFSET_TRANSITION_POINTS) and \
             current_idx <= (c_info['end_idx'] + SMOOTH_OFFSET_TRANSITION_POINTS):
            active_corner_for_direction = c_info
            
    if active_corner_for_direction:
        # Vector from current point to corner center
        vec_to_center_x = active_corner_for_direction['center_x'] - path_x[current_idx]
        vec_to_center_y = active_corner_for_direction['center_y'] - path_y[current_idx]

        # Dot product to check if perp_vector points towards or away from the center
        # If dot product is positive, perp_vector points towards center, so we need to reverse it
        dot_product = perp_x * vec_to_center_x + perp_y * vec_to_center_y
        
        if dot_product > 0: # This means perp_x, perp_y points towards the center of the corner
            offset_direction_multiplier = -1 # So we need to reverse the offset direction to go outward

    return perp_x * offset_direction_multiplier, perp_y * offset_direction_multiplier


for i in range(len(x)): 
    current_x_a = x[i]
    current_y_a = y[i]
    
    # 1. Determine target offset amount (smoothed from straight to corner values)
    target_offset_amount = STRAIGHT_LATERAL_OFFSET 
    
    # Find the corner section that this point is within or transitioning to/from
    active_corner_section = None
    for c_section in corner_sections: # Using raw corner_sections list
        start_idx = c_section[0]
        end_idx = c_section[-1]

        # Check if current point is within the *extended* influence zone of this corner
        if i >= (start_idx - SMOOTH_OFFSET_TRANSITION_POINTS) and \
           i <= (end_idx + SMOOTH_OFFSET_TRANSITION_POINTS):
            active_corner_section = c_section
            break
            
    if active_corner_section:
        corner_start_idx = active_corner_section[0]
        corner_end_idx = active_corner_section[-1]

        # Calculate progress within the combined transition/corner zone
        if i < corner_start_idx: # Transitioning into the corner
            progress = (i - (corner_start_idx - SMOOTH_OFFSET_TRANSITION_POINTS)) / SMOOTH_OFFSET_TRANSITION_POINTS
        elif i > corner_end_idx: # Transitioning out of the corner
            progress = ((corner_end_idx + SMOOTH_OFFSET_TRANSITION_POINTS) - i) / SMOOTH_OFFSET_TRANSITION_POINTS
        else: # Inside the main corner section
            progress = 1.0 # Full corner offset

        progress = max(0.0, min(1.0, progress)) # Clamp progress between 0 and 1
        
        target_offset_amount = STRAIGHT_LATERAL_OFFSET + (CORNER_TARGET_LATERAL_OFFSET - STRAIGHT_LATERAL_OFFSET) * progress
    
    # 2. Get the correctly oriented OUTWARD normal vector for this point
    outward_normal_x, outward_normal_y = get_outward_normal(i, x, y, corner_details)
    
    # 3. Apply the target offset using the outward normal
    raw_x_b[i] = current_x_a + outward_normal_x * target_offset_amount
    raw_y_b[i] = current_y_a + outward_normal_y * target_offset_amount


# --- Apply Moving Average Smoothing to the raw path ---
x_car_b = [0.0] * len(x)
y_car_b = [0.0] * len(y)

# Moving average window for final path smoothing
FINAL_SMOOTHING_WINDOW = 12 # Tune this: larger = smoother, but can cut corners more or delay transitions

for i in range(len(raw_x_b)):
    x_sum = 0
    y_sum = 0
    count = 0
    
    start_window = max(0, i - FINAL_SMOOTHING_WINDOW // 2)
    end_window = min(len(raw_x_b) - 1, i + FINAL_SMOOTHING_WINDOW // 2)
    
    for j in range(start_window, end_window + 1):
        x_sum += raw_x_b[j]
        y_sum += raw_y_b[j]
        count += 1
        
    x_car_b[i] = x_sum / count
    y_car_b[i] = y_sum / count

# Create the JavaScript output
js_content = "const carACoordinates = [\n"

for i in range(len(x)):
    js_content += f"    {{ x: {x[i]:.3f}, y: {y[i]:.3f}, time: {times[i]:.3f} }}"
    if i < len(x) - 1:
        js_content += ","
    js_content += "\n"

js_content += "];\n\n"

js_content += "const carBCoordinates = [\n"

for i in range(len(x_car_b)):
    js_content += f"    {{ x: {x_car_b[i]:.3f}, y: {y_car_b[i]:.3f}, time: {times_car_b[i]:.3f} }}"
    if i < len(x_car_b) - 1:
        js_content += ","
    js_content += "\n"

js_content += "];\n\n"

js_content += "export { carACoordinates as kartCoordinates, carACoordinates as kartLineCoordinates, carBCoordinates };"

# Write to file
with open('racing_line_with_time.js', 'w') as f:
    f.write(js_content)

print(f"\nGenerated racing line with {len(x)} points")
print(f"Car A total lap time: {current_time:.3f} seconds")
print(f"Car B total lap time: {current_time_b:.3f} seconds")
print(f"Time difference: {abs(current_time_b - current_time):.3f} seconds")
print(f"Output saved to 'racing_line_with_time.js'")

# Print some statistics
total_distance = sum(distances)
total_distance_b = sum(distances)  # Car B travels same racing line distance
average_speed = total_distance / current_time
average_speed_b = total_distance_b / current_time_b

print(f"\nCar A Statistics:")
print(f"  Total distance: {total_distance:.1f}m")
print(f"  Average speed: {average_speed:.1f}m/s ({average_speed * 3.6:.1f}km/h)")

print(f"\nCar B Statistics:")
print(f"  Total distance: {total_distance_b:.1f}m")
print(f"  Average speed: {average_speed_b:.1f}m/s ({average_speed_b * 3.6:.1f}km/h)")

# Calculate acceleration statistics
accelerations = []
for i in range(len(distances)):
    if times[i+1] - times[i] > 0:
        accel = (actual_speeds[i+1] - actual_speeds[i]) / (times[i+1] - times[i])
    else:
        accel = 0
    accelerations.append(accel)

if accelerations:
    max_accel = max(accelerations)
    max_decel = min(accelerations)
    print(f"Maximum acceleration: {max_accel:.2f}m/s²")
    print(f"Maximum deceleration: {max_decel:.2f}m/s²")

# Count segment types
straight_count = sum(1 for i in range(len(distances)) if i not in corner_indices and i not in transition_segments and i not in braking_zone_segments)
braking_zone_count = len(braking_zone_segments)
corner_entry_count = len(corner_entry_segments)
corner_exit_count = len(corner_exit_segments)
transition_count = len(transition_segments)
acceleration_zone_count = len(acceleration_zone_segments)
other_corner_count = sum(1 for i in range(len(distances)) if i in corner_indices and i not in transition_segments and i not in corner_entry_segments and i not in corner_exit_segments and i not in braking_zone_segments)

print(f"Segment breakdown:")
print(f"  Straights: {straight_count} segments")
print(f"  Braking Zones: {braking_zone_count} segments")
print(f"  Corner Entry: {corner_entry_count} segments")
print(f"  Transitions (Apex): {transition_count} segments")
print(f"  Corner Exit: {corner_exit_count} segments")
print(f"  Acceleration Zones: {acceleration_zone_count} segments")
print(f"  Other Corners: {other_corner_count} segments")
