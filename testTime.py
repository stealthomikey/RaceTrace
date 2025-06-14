import numpy as np
import pandas as pd

# --- Configuration for Car A ---
TIME_MULTIPLIER_STRAIGHT_A = 0.020 / 4  # 0.005, ~200 m/s
TIME_MULTIPLIER_CORNER_A = 0.080 / 4    # 0.020, ~50 m/s
TIME_MULTIPLIER_APEX_TRANSITION_A = 0.050 / 4  # 0.0125, ~80 m/s
TIME_MULTIPLIER_EXIT_ACCEL_A = 0.025 / 4      # 0.00625, ~160 m/s
MAX_ACCELERATION_A = 0.05 * 4  # 0.2 m/s per meter
MAX_DECELERATION_A = 0.1 * 4   # 0.4 m/s per meter
INITIAL_SPEED_A = 1 / (max(TIME_MULTIPLIER_CORNER_A, TIME_MULTIPLIER_APEX_TRANSITION_A) * 1.2)

# --- Configuration for Car B ---
TIME_MULTIPLIER_STRAIGHT_B = 0.022 / 4  # 0.0055, ~181.82 m/s
TIME_MULTIPLIER_CORNER_B = 0.090 / 4    # 0.0225, ~44.44 m/s
TIME_MULTIPLIER_APEX_TRANSITION_B = 0.060 / 4  # 0.015, ~66.67 m/s
TIME_MULTIPLIER_EXIT_ACCEL_B = 0.028 / 4      # 0.007, ~142.86 m/s
MAX_ACCELERATION_B = 0.03 * 4  # 0.12 m/s per meter
MAX_DECELERATION_B = 0.15 * 4  # 0.6 m/s per meter
INITIAL_SPEED_B = 1 / (max(TIME_MULTIPLIER_CORNER_B, TIME_MULTIPLIER_APEX_TRANSITION_B) * 1.2)

# --- Offset Configuration ---
STRAIGHT_OFFSET = -0.5  # Meters to offset Car B on straights (left)
CORNER_OFFSET_BASE = -1.2  # Base offset for wider corners
MAX_OFFSET = 1.5  # Maximum offset to stay on track
BEZIER_CONTROL_SCALE = 1.5  # Scale control points for wider/shallower corners
OFFSET_SMOOTHING_RADIUS = 5  # Smooth offset transitions

# --- Smoothing Configuration ---
SMOOTHING_WINDOW_RADIUS = 40  # For speed multipliers

def generate_js_output(coordinates, car_name):
    js_coordinates = []
    for point in coordinates:
        js_coordinates.append(f"    {{ x: {point['x']:.3f}, y: {point['y']:.3f}, time: {point['time']:.3f} }}")
    js_string = f"const {car_name}Coordinates = [\n" + ",\n".join(js_coordinates) + \
                f"\n];\nexport {{ {car_name}Coordinates as kartCoordinates, {car_name}Coordinates as kartLineCoordinates }};"
    return js_string

def write_js_file(coordinates, car_name, filename):
    js_content = generate_js_output(coordinates, car_name)
    try:
        with open(filename, 'w') as f:
            f.write(js_content)
        print(f"Successfully wrote {filename}")
    except Exception as e:
        print(f"Error writing {filename}: {e}")

def calculate_angle(p1, p2, p3):
    v1 = np.array([p1[0] - p2[0], p1[1] - p2[1]])
    v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
    mag1 = np.sqrt(v1.dot(v1))
    mag2 = np.sqrt(v2.dot(v2))
    if mag1 == 0 or mag2 == 0: return 0.0
    cos_angle = np.dot(v1, v2) / (mag1 * mag2)
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle_deg = np.degrees(np.arccos(cos_angle))
    return min(angle_deg, 180.0 - angle_deg)

def calculate_offset_point(p1, p2, offset_distance):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    length = np.sqrt(dx**2 + dy**2)
    if length == 0:
        return p1[0], p1[1]
    dx, dy = dx / length, dy / length
    perp_dx, perp_dy = dy, -dx  # Left offset
    return p1[0] + perp_dx * offset_distance, p1[1] + perp_dy * offset_distance

def quadratic_bezier(t, p0, p1, p2):
    """Calculate point on quadratic Bézier curve at parameter t (0 to 1)."""
    x = (1-t)**2 * p0[0] + 2*(1-t)*t * p1[0] + t**2 * p2[0]
    y = (1-t)**2 * p0[1] + 2*(1-t)*t * p1[1] + t**2 * p2[1]
    return x, y

def main():
    try:
        df = pd.read_csv('racing_line.csv', comment='#', header=None, names=['x_m', 'y_m'])
        if df.empty: print("Error: 'racing_line.csv' is empty."); return
        df['x_m'] = pd.to_numeric(df['x_m'], errors='coerce')
        df['y_m'] = pd.to_numeric(df['y_m'], errors='coerce')
        df.dropna(subset=['x_m', 'y_m'], inplace=True)
        if df.empty: print("Error: No valid numeric data."); return
        data = list(zip(df['x_m'], df['y_m']))
    except FileNotFoundError: print("Error: 'racing_line.csv' not found."); return
    except Exception as e: print(f"Error reading CSV: {e}"); return

    if not data: print("Error: No data loaded."); return
    x_coords, y_coords = zip(*data)
    num_points = len(x_coords)

    if num_points == 0:
        print("No coordinates.")
        write_js_file([], "carA", "carA.js")
        write_js_file([], "carB", "carB.js")
        return

    coordinates_a = [{'x': x_coords[0], 'y': y_coords[0], 'time': 0.0}]
    coordinates_b = [{'x': x_coords[0], 'y': y_coords[0], 'time': 0.0}]

    if num_points <= 1:
        write_js_file(coordinates_a, "carA", "carA.js")
        write_js_file(coordinates_b, "carB", "carB.js")
        return

    # Calculate angles to identify corners
    n_angle_calc, angle_sharpness_threshold = 10, 5
    calculated_angles, angle_calc_indices = [], []
    if num_points > 2 * n_angle_calc:
        for i in range(n_angle_calc, num_points - n_angle_calc):
            angle = calculate_angle(data[i-n_angle_calc], data[i], data[i+n_angle_calc])
            calculated_angles.append(angle)
            angle_calc_indices.append(i)

    # Identify corner segments and assign angles
    all_corner_segments = set()
    segment_angles = [0.0] * (num_points - 1)
    if calculated_angles:
        for i in range(num_points - 1):
            original_point_idx = min(angle_calc_indices, key=lambda idx: abs(idx - i))
            idx_in_list = angle_calc_indices.index(original_point_idx)
            segment_angles[i] = calculated_angles[idx_in_list]
            if calculated_angles[idx_in_list] > angle_sharpness_threshold:
                all_corner_segments.add(i)

    # Group corner segments
    corner_sections, current_section = [], []
    if all_corner_segments:
        for i in range(num_points - 1):
            if i in all_corner_segments:
                current_section.append(i)
            else:
                if current_section:
                    corner_sections.append(list(current_section))
                    current_section = []
        if current_section:
            corner_sections.append(list(current_section))

    # Assign apex and exit zones
    original_transition_length = 4
    apex_transition_indices, exit_accel_indices = set(), set()
    for section in corner_sections:
        if len(section) >= original_transition_length:
            start_offset = (len(section) - original_transition_length) // 2
            num_apex = original_transition_length // 2
            for k in range(original_transition_length):
                actual_segment_idx = section[start_offset + k]
                if k < num_apex:
                    apex_transition_indices.add(actual_segment_idx)
                else:
                    exit_accel_indices.add(actual_segment_idx)

    # Assign desired multipliers for Car A
    num_segments = num_points - 1
    ideal_multipliers_a = [TIME_MULTIPLIER_STRAIGHT_A] * num_segments
    for i in range(num_segments):
        if i in all_corner_segments:
            ideal_multipliers_a[i] = TIME_MULTIPLIER_CORNER_A
            if i in exit_accel_indices:
                ideal_multipliers_a[i] = TIME_MULTIPLIER_EXIT_ACCEL_A
            elif i in apex_transition_indices:
                ideal_multipliers_a[i] = TIME_MULTIPLIER_APEX_TRANSITION_A

    # Assign desired multipliers for Car B
    ideal_multipliers_b = [TIME_MULTIPLIER_STRAIGHT_B] * num_segments
    for i in range(num_segments):
        if i in all_corner_segments:
            ideal_multipliers_b[i] = TIME_MULTIPLIER_CORNER_B
            if i in exit_accel_indices:
                ideal_multipliers_b[i] = TIME_MULTIPLIER_EXIT_ACCEL_B
            elif i in apex_transition_indices:
                ideal_multipliers_b[i] = TIME_MULTIPLIER_APEX_TRANSITION_B

    # Smooth desired multipliers
    desired_multipliers_a = [0.0] * num_segments
    desired_multipliers_b = [0.0] * num_segments
    if num_segments > 0:
        if SMOOTHING_WINDOW_RADIUS <= 0:
            desired_multipliers_a = list(ideal_multipliers_a)
            desired_multipliers_b = list(ideal_multipliers_b)
        else:
            for i in range(num_segments):
                start = max(0, i - SMOOTHING_WINDOW_RADIUS)
                end = min(num_segments - 1, i + SMOOTHING_WINDOW_RADIUS)
                window_a = ideal_multipliers_a[start:end + 1]
                window_b = ideal_multipliers_b[start:end + 1]
                desired_multipliers_a[i] = sum(window_a) / len(window_a) if window_a else ideal_multipliers_a[i]
                desired_multipliers_b[i] = sum(window_b) / len(window_b) if window_b else ideal_multipliers_b[i]

    # Generate Car B's path with wider/shallower corners
    coordinates_b = [coordinates_b[0]]
    current_segment = 0
    for section in corner_sections + [[num_segments]]:  # Include final segment
        # Process straights before this corner section
        while current_segment < num_segments and current_segment not in all_corner_segments:
            seg_x1, seg_y1 = x_coords[current_segment], y_coords[current_segment]
            seg_x2, seg_y2 = x_coords[current_segment+1], y_coords[current_segment+1]
            offset_x, offset_y = calculate_offset_point((seg_x1, seg_y1), (seg_x2, seg_y2), STRAIGHT_OFFSET)
            coordinates_b.append({'x': offset_x, 'y': offset_y, 'time': 0.0})  # Time set later
            current_segment += 1

        # Process corner section with Bézier curve
        if current_segment < num_segments and current_segment in all_corner_segments:
            section_indices = [i for i in section if i in all_corner_segments]
            if section_indices:
                start_idx = section_indices[0]
                mid_idx = section_indices[len(section_indices)//2]
                end_idx = section_indices[-1] + 1 if section_indices[-1] + 1 < num_points else section_indices[-1]
                
                p0 = np.array([x_coords[start_idx], y_coords[start_idx]])
                p2 = np.array([x_coords[end_idx], y_coords[end_idx]])
                pm = np.array([x_coords[mid_idx], y_coords[mid_idx]])
                v1 = p2 - p0
                length = np.sqrt(v1.dot(v1))
                if length > 0:
                    v1 = v1 / length
                    perp_v1 = np.array([v1[1], -v1[0]])  # Perpendicular left
                    angle_factor = min(segment_angles[mid_idx] / angle_sharpness_threshold, 2.0)
                    offset_dist = min(CORNER_OFFSET_BASE * angle_factor, MAX_OFFSET)
                    p1 = pm + perp_v1 * offset_dist * BEZIER_CONTROL_SCALE
                    num_bezier_points = len(section_indices) + 1
                    for t in np.linspace(0, 1, num_bezier_points):
                        if t == 0:
                            continue
                        bx, by = quadratic_bezier(t, p0, p1, p2)
                        coordinates_b.append({'x': bx, 'y': by, 'time': 0.0})
                else:
                    for i in section_indices:
                        offset_x, offset_y = calculate_offset_point(
                            (x_coords[i], y_coords[i]),
                            (x_coords[i+1], y_coords[i+1]),
                            min(CORNER_OFFSET_BASE, MAX_OFFSET)
                        )
                        coordinates_b.append({'x': offset_x, 'y': offset_y, 'time': 0.0})
                current_segment = end_idx

    # Ensure coordinates_b has same length as coordinates_a
    while len(coordinates_b) < num_points and current_segment < num_segments:
        offset_x, offset_y = calculate_offset_point(
            (x_coords[current_segment], y_coords[current_segment]),
            (x_coords[current_segment+1], y_coords[current_segment+1]),
            STRAIGHT_OFFSET
        )
        coordinates_b.append({'x': offset_x, 'y': offset_y, 'time': 0.0})
        current_segment += 1

    # Calculate speeds and times
    actual_speeds_a = [0.0] * num_segments
    actual_speeds_b = [0.0] * num_segments
    current_speed_a = INITIAL_SPEED_A
    current_speed_b = INITIAL_SPEED_B
    current_time_a = 0.0
    current_time_b = 0.0

    for i in range(num_segments):
        seg_x1, seg_y1 = x_coords[i], y_coords[i]
        seg_x2, seg_y2 = x_coords[i+1], y_coords[i+1]
        distance = np.sqrt((seg_x2 - seg_x1)**2 + (seg_y2 - seg_y1)**2)

        if distance == 0:
            actual_speeds_a[i] = current_speed_a
            actual_speeds_b[i] = current_speed_b
            coordinates_a.append({'x': seg_x2, 'y': seg_y2, 'time': round(current_time_a, 3)})
            coordinates_b[i + 1]['time'] = round(current_time_b, 3)
            continue

        # Car A: Update speed
        target_speed_a = 1 / desired_multipliers_a[i]
        max_speed_increase_a = MAX_ACCELERATION_A * distance
        max_speed_decrease_a = MAX_DECELERATION_A * distance
        if target_speed_a > current_speed_a:
            actual_speeds_a[i] = min(target_speed_a, current_speed_a + max_speed_increase_a)
        else:
            actual_speeds_a[i] = max(target_speed_a, current_speed_a - max_speed_decrease_a)
        current_speed_a = actual_speeds_a[i]
        time_for_segment_a = distance / current_speed_a
        current_time_a += time_for_segment_a

        # Car B: Update speed
        target_speed_b = 1 / desired_multipliers_b[i]
        max_speed_increase_b = MAX_ACCELERATION_B * distance
        max_speed_decrease_b = MAX_DECELERATION_B * distance
        if target_speed_b > current_speed_b:
            actual_speeds_b[i] = min(target_speed_b, current_speed_b + max_speed_increase_b)
        else:
            actual_speeds_b[i] = max(target_speed_b, current_speed_b - max_speed_decrease_b)
        current_speed_b = actual_speeds_b[i]
        time_for_segment_b = distance / current_speed_b
        current_time_b += time_for_segment_b

        # Append coordinates
        coordinates_a.append({'x': seg_x2, 'y': seg_y2, 'time': round(current_time_a, 3)})
        coordinates_b[i + 1]['time'] = round(current_time_b, 3)

    # Write to separate JS files
    write_js_file(coordinates_a, "carA", "carA.js")
    write_js_file(coordinates_b, "carB", "carB.js")

if __name__ == '__main__':
    main()
