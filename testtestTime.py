import numpy as np
import pandas as pd
import math

# --- Configuration ---
# Speed multipliers (tuned for ~90s lap times)
STRAIGHT_SPEED_MULTIPLIER = 0.08   # ~125 m/s on straights (450 km/h)
CORNER_SPEED_MULTIPLIER = 0.025     # ~40 m/s in corners (144 km/h)
TRANSITION_SPEED_MULTIPLIER = 0.015 # ~67 m/s in transitions (240 km/h)

# Car differences
CAR_A_SPEED_FACTOR = 1.0    # Base speed
CAR_B_SPEED_FACTOR = 0.9    # 10% slower overall

# Track offset for Car B (meters)
TRACK_OFFSET = -1.5  # Negative = left side, positive = right side

# Physics  
MAX_ACCELERATION = 8.0   # m/s² - higher for faster speed changes
MAX_DECELERATION = 12.0  # m/s² - higher for better braking into corners

# Corner detection
ANGLE_THRESHOLD = 10.0  # Degrees - sharper than this = corner
LOOKAHEAD_DISTANCE = 5  # Points to look ahead for angle calculation

def calculate_distance(p1, p2):
    """Calculate distance between two points."""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def calculate_angle_at_point(points, index, lookahead=LOOKAHEAD_DISTANCE):
    """Calculate the turning angle at a point."""
    if index < lookahead or index >= len(points) - lookahead:
        return 0.0
    
    p1 = points[index - lookahead]
    p2 = points[index]
    p3 = points[index + lookahead]
    
    # Calculate vectors
    v1 = np.array([p1[0] - p2[0], p1[1] - p2[1]])
    v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
    
    # Calculate angle
    try:
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = math.degrees(math.acos(cos_angle))
        return 180.0 - angle  # Convert to turning angle
    except:
        return 0.0

def offset_point(p1, p2, offset_distance):
    """Calculate offset point perpendicular to the line p1->p2."""
    if calculate_distance(p1, p2) == 0:
        return p1
    
    # Direction vector
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = math.sqrt(dx*dx + dy*dy)
    
    # Normalize
    dx /= length
    dy /= length
    
    # Perpendicular vector (rotate 90° left)
    perp_x = -dy
    perp_y = dx
    
    return (p1[0] + perp_x * offset_distance, p1[1] + perp_y * offset_distance)

def smooth_path(points, smoothing_factor=0.3):
    """Apply simple smoothing to a path."""
    if len(points) < 3:
        return points
    
    smoothed = [points[0]]  # Keep first point
    
    for i in range(1, len(points) - 1):
        # Weighted average with neighbors
        prev_pt = points[i-1]
        curr_pt = points[i]
        next_pt = points[i+1]
        
        new_x = curr_pt[0] * (1 - smoothing_factor) + (prev_pt[0] + next_pt[0]) * smoothing_factor / 2
        new_y = curr_pt[1] * (1 - smoothing_factor) + (prev_pt[1] + next_pt[1]) * smoothing_factor / 2
        
        smoothed.append((new_x, new_y))
    
    smoothed.append(points[-1])  # Keep last point
    return smoothed

def generate_car_b_path(original_points):
    """Generate Car B's path with wider corners."""
    if len(original_points) < 3:
        return original_points
    
    # Create offset path
    offset_points = []
    
    for i in range(len(original_points)):
        if i == 0:
            # First point - offset based on next segment
            offset_pt = offset_point(original_points[i], original_points[i+1], TRACK_OFFSET)
        elif i == len(original_points) - 1:
            # Last point - offset based on previous segment
            offset_pt = offset_point(original_points[i-1], original_points[i], TRACK_OFFSET)
        else:
            # Middle point - offset based on average direction
            prev_offset = offset_point(original_points[i-1], original_points[i], TRACK_OFFSET)
            next_offset = offset_point(original_points[i], original_points[i+1], TRACK_OFFSET)
            offset_pt = ((prev_offset[0] + next_offset[0]) / 2, (prev_offset[1] + next_offset[1]) / 2)
        
        offset_points.append(offset_pt)
    
    # Smooth the offset path to make corners more natural
    return smooth_path(offset_points, 0.2)

def calculate_speeds_and_times(points, speed_factor=1.0):
    """Calculate speeds and times for a racing line."""
    if len(points) < 2:
        return [{'x': points[0][0], 'y': points[0][1], 'time': 0.0}] if points else []
    
    # Analyze corners
    corner_factors = []
    corner_types = []
    for i in range(len(points)):
        angle = calculate_angle_at_point(points, i)
        
        if angle > ANGLE_THRESHOLD:
            # Sharp corner
            corner_factors.append(CORNER_SPEED_MULTIPLIER)
            corner_types.append("CORNER")
        elif angle > ANGLE_THRESHOLD / 3:
            # Gentle corner/transition
            corner_factors.append(TRANSITION_SPEED_MULTIPLIER)
            corner_types.append("TRANSITION")
        else:
            # Straight
            corner_factors.append(STRAIGHT_SPEED_MULTIPLIER)
            corner_types.append("STRAIGHT")
    
    # Debug output
    corner_count = sum(1 for t in corner_types if t == "CORNER")
    straight_count = sum(1 for t in corner_types if t == "STRAIGHT")
    transition_count = sum(1 for t in corner_types if t == "TRANSITION")
    print(f"  Track analysis: {straight_count} straights, {transition_count} transitions, {corner_count} corners")
    
    # Calculate target speeds (inverse of time multiplier)
    target_speeds = [1.0 / (factor * speed_factor) for factor in corner_factors]
    
    # Apply physics constraints
    actual_speeds = [target_speeds[0]]
    for i in range(1, len(target_speeds)):
        if i >= len(points) - 1:
            break
            
        distance = calculate_distance(points[i-1], points[i])
        if distance == 0:
            actual_speeds.append(actual_speeds[-1])
            continue
            
        target = target_speeds[i]
        current = actual_speeds[-1]
        
        # Calculate maximum possible speed change based on physics
        # Using v² = u² + 2as, so Δv = √(2*a*s) for max change
        if target > current:
            # Accelerating
            max_speed_increase = math.sqrt(2 * MAX_ACCELERATION * distance) if distance > 0 else 0
            new_speed = min(target, current + max_speed_increase)
        else:
            # Decelerating (braking for corners)
            max_speed_decrease = math.sqrt(2 * MAX_DECELERATION * distance) if distance > 0 else 0
            new_speed = max(target, current - max_speed_decrease)
            
            actual_speeds.append(new_speed)
    
    # Apply additional smoothing to make speed changes more gradual
    smoothed_speeds = [actual_speeds[0]]
    for i in range(1, len(actual_speeds)):
        # Weighted average with previous speed for smoother transitions
        smoothed_speed = 0.7 * actual_speeds[i] + 0.3 * smoothed_speeds[i-1]
        smoothed_speeds.append(smoothed_speed)
    
    actual_speeds = smoothed_speeds
    
    # Calculate times
    coordinates = []
    current_time = 0.0
    
    coordinates.append({'x': points[0][0], 'y': points[0][1], 'time': 0.0})
    
    for i in range(1, len(points)):
        distance = calculate_distance(points[i-1], points[i])
        if distance == 0 or i-1 >= len(actual_speeds):
            time_delta = 0.0
        else:
            speed = actual_speeds[i-1]
            time_delta = distance / speed if speed > 0 else 0.0
            
        current_time += time_delta
        coordinates.append({
            'x': round(points[i][0], 3),
            'y': round(points[i][1], 3),
            'time': round(current_time, 3)
        })
    
    return coordinates

def write_js_file(coordinates, car_name, filename):
    """Write coordinates to JavaScript file."""
    js_coordinates = []
    for i, point in enumerate(coordinates):
        comma = "," if i < len(coordinates) - 1 else ""
        js_coordinates.append(f"    {{ x: {point['x']:.3f}, y: {point['y']:.3f}, time: {point['time']:.3f} }}{comma}")
    
    js_content = f"""const {car_name}Coordinates = [
{chr(10).join(js_coordinates)}
];

export {{ {car_name}Coordinates as kartCoordinates, {car_name}Coordinates as kartLineCoordinates }};"""
    
    try:
        with open(filename, 'w') as f:
            f.write(js_content)
        print(f"Successfully wrote {filename}")
        print(f"  - {len(coordinates)} points")
        print(f"  - Total time: {coordinates[-1]['time']:.3f}s")
        print(f"  - Max speed: ~{1/min([STRAIGHT_SPEED_MULTIPLIER, CORNER_SPEED_MULTIPLIER, TRANSITION_SPEED_MULTIPLIER]):.1f} m/s")
    except Exception as e:
        print(f"Error writing {filename}: {e}")

def main():
    # Read the racing line CSV
    try:
        df = pd.read_csv('racing_line.csv', comment='#', header=None, names=['x_m', 'y_m'])
        
        if df.empty:
            print("Error: 'racing_line.csv' is empty.")
            return
            
        # Clean the data
        df['x_m'] = pd.to_numeric(df['x_m'], errors='coerce')
        df['y_m'] = pd.to_numeric(df['y_m'], errors='coerce')
        df.dropna(subset=['x_m', 'y_m'], inplace=True)
        
        if df.empty:
            print("Error: No valid numeric data found.")
            return
            
        original_points = list(zip(df['x_m'], df['y_m']))
        
    except FileNotFoundError:
        print("Error: 'racing_line.csv' not found.")
        return
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return
    
    print(f"Loaded {len(original_points)} points from racing_line.csv")
    
    if len(original_points) < 2:
        print("Error: Need at least 2 points to generate racing lines.")
        return
    
    # Generate Car A (follows original line)
    print("\nGenerating Car A (original line)...")
    car_a_coords = calculate_speeds_and_times(original_points, CAR_A_SPEED_FACTOR)
    
    # Generate Car B (wider/different line)
    print("Generating Car B (offset line)...")
    car_b_points = generate_car_b_path(original_points)
    car_b_coords = calculate_speeds_and_times(car_b_points, CAR_B_SPEED_FACTOR)
    
    # Write output files
    print("\nWriting output files...")
    write_js_file(car_a_coords, "carA", "carA.js")
    write_js_file(car_b_coords, "carB", "carB.js")
    
    print(f"\nRace Results:")
    print(f"Car A time: {car_a_coords[-1]['time']:.3f}s")
    print(f"Car B time: {car_b_coords[-1]['time']:.3f}s")
    print(f"Difference: {abs(car_a_coords[-1]['time'] - car_b_coords[-1]['time']):.3f}s")

if __name__ == '__main__':
    main()
