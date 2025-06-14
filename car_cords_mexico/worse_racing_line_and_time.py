import csv
import math

def calculate_distance(p1, p2):
    """Calculates the Euclidean distance between two points."""
    return math.sqrt((p2['x'] - p1['x'])**2 + (p2['y'] - p1['y'])**2)

def calculate_angle(p_prev, p_curr, p_next):
    """
    Calculates the internal angle of the turn at p_curr formed by the vectors p_prev->p_curr and p_curr->p_next.
    Returns the angle in degrees (0 for straight, 180 for a full reversal).
    """
    if p_prev is None or p_next is None:
        return 0 # No meaningful angle for start/end points

    vec1_x = p_curr['x'] - p_prev['x']
    vec1_y = p_curr['y'] - p_prev['y']
    vec2_x = p_next['x'] - p_curr['x']
    vec2_y = p_next['y'] - p_curr['y']

    dot_product = vec1_x * vec2_x + vec1_y * vec2_y
    magnitude1 = math.sqrt(vec1_x**2 + vec1_y**2)
    magnitude2 = math.sqrt(vec2_x**2 + vec2_y**2)

    if magnitude1 == 0 or magnitude2 == 0:
        return 0 # Avoid division by zero

    try:
        angle_rad = math.acos(min(max(dot_product / (magnitude1 * magnitude2), -1.0), 1.0)) # Clamp for float precision
        return math.degrees(angle_rad)
    except ValueError:
        return 0

def get_turn_direction_for_offset(p_prev, p_curr, p_next):
    """
    Determines if the turn at p_curr is a left or right turn, for the purpose of offsetting outwards.
    Returns 1 for a left turn (offset to the right), -1 for a right turn (offset to the left), 0 for straight.
    This dictates which 'side' of the ideal line the slower car will deviate.
    """
    if p_prev is None or p_next is None:
        return 0

    # Vector 1: p_prev to p_curr
    v1_x = p_curr['x'] - p_prev['x']
    v1_y = p_curr['y'] - p_prev['y']

    # Vector 2: p_curr to p_next
    v2_x = p_next['x'] - p_curr['x']
    v2_y = p_next['y'] - p_curr['y']

    # 2D Cross Product (v1_x * v2_y - v1_y * v2_x)
    cross_product_z = v1_x * v2_y - v1_y * v2_x

    if cross_product_z > 0.001: 
        return 1  # Left turn
    elif cross_product_z < -0.001:
        return -1 # Right turn
    else:
        return 0  # Straight


def offset_point_smooth_directional(p_curr, p_next, offset_magnitude, turn_direction):
    """
    Offsets p_curr's position to `p_next`'s position perpendicular to the segment (p_curr, p_next),
    considering the turn direction to always push outwards.
    This produces a smoother offset.
    
    Args:
        p_curr (dict): The current point {'x', 'y'}.
        p_next (dict): The next point {'x', 'y'}. Used to determine segment direction.
        offset_magnitude (float): How far to offset.
        turn_direction (int): 1 for left turn, -1 for right turn, 0 for straight.

    Returns:
        tuple: (new_x, new_y) for the deviated point.
    """
    if p_next is None: # Last point, no forward segment
        return p_curr['x'], p_curr['y']

    dx = p_next['x'] - p_curr['x']
    dy = p_next['y'] - p_curr['y']

    length = math.sqrt(dx**2 + dy**2)
    if length == 0:
        return p_curr['x'], p_curr['y'] # No movement

    unit_dx = dx / length
    unit_dy = dy / length

    # If it's a left turn (turn_direction = 1), we want to push to the "right" side (outwards).
    # If it's a right turn (turn_direction = -1), we want to push to the "left" side (outwards).
    # This is achieved by multiplying the perpendicular vector by `turn_direction`.

    offset_x = unit_dy * offset_magnitude * turn_direction
    offset_y = -unit_dx * offset_magnitude * turn_direction
    
    return p_curr['x'] + offset_x, p_curr['y'] + offset_y


def get_effective_speed(
    points, current_idx, num_points, 
    straight_speed, corner_speed, 
    braking_distance, acceleration_distance, 
    max_corner_angle
):
    """
    Calculates the effective speed for the current segment, considering upcoming corners,
    braking zones, and acceleration zones.

    Args:
        points (list): List of all raw {'x', 'y'} points.
        current_idx (int): Index of the current point (start of the segment).
        num_points (int): Total number of points.
        straight_speed (float): Max speed on straights.
        corner_speed (float): Speed within a corner.
        braking_distance (float): How many units before a corner to start braking.
        acceleration_distance (float): How many units after a corner to reach straight speed.
        max_corner_angle (float): Angle threshold for a "sharp" corner (degrees).

    Returns:
        float: The calculated effective speed for the segment from current_idx to current_idx+1.
    """
    p_curr = points[current_idx]
    
    p_next_for_speed_calc = points[current_idx + 1] if current_idx + 1 < num_points else None

    if p_next_for_speed_calc is None: # Last point, no segment after it
        return straight_speed 

    # --- Check for upcoming corners for braking ---
    distance_to_next_corner = float('inf')
    path_dist_to_look_ahead = 0.0 # Initialize here
    
    for i in range(current_idx, num_points - 1): # Iterate from current point to find next corner
        p1 = points[i]
        p2 = points[i+1]
        segment_len = calculate_distance(p1, p2)
        
        if i + 2 < num_points: # Need at least two more points to form an angle
            angle = calculate_angle(p1, p2, points[i+2])
            if angle > max_corner_angle:
                distance_to_next_corner = path_dist_to_look_ahead # Distance *to the start* of the corner
                break
        path_dist_to_look_ahead += segment_len # Accumulate path distance


    # --- Determine Speed ---
    if distance_to_next_corner < braking_distance:
        # We are in a braking zone
        braking_progress = 1 - (distance_to_next_corner / braking_distance) 
        return straight_speed - (straight_speed - corner_speed) * min(max(braking_progress, 0.0), 1.0) 
    
    # Check if we are currently *in* a corner (based on angle of next segment)
    if current_idx + 2 < num_points: 
        angle_of_next_segment = calculate_angle(p_curr, p_next_for_speed_calc, points[current_idx + 2])
        if angle_of_next_segment > max_corner_angle:
            return corner_speed 

    # Check for acceleration zone after a corner
    distance_from_last_corner = float('inf')
    path_dist_backward = 0.0 # Initialize here
    
    for i in range(current_idx, 0, -1): # Iterate backwards from current point
        p1 = points[i]
        p2 = points[i-1]
        segment_len = calculate_distance(p1, p2)
        
        if i - 2 >= 0: # Need at least two previous points
            angle = calculate_angle(points[i-2], p2, p1)
            if angle > max_corner_angle:
                distance_from_last_corner = path_dist_backward # Distance from the corner *exit*
                break
        path_dist_backward += segment_len


    if distance_from_last_corner < acceleration_distance:
        # We are in an acceleration zone
        accel_progress = 1 - (distance_from_last_corner / acceleration_distance) 
        return corner_speed + (straight_speed - corner_speed) * min(max(accel_progress, 0.0), 1.0) 

    # Default: Straight speed
    return straight_speed


def get_smoothed_offset_magnitude(
    points, current_idx, num_points, 
    offset_factor_straight, offset_factor_corner, 
    max_corner_angle, blend_distance_offset
):
    """
    Calculates the smoothed offset magnitude for the current point, blending between
    straight and corner offsets.
    """
    p_curr = points[current_idx]
    
    # Find distance to next significant corner
    dist_to_next_corner = float('inf')
    path_dist_to_look_ahead = 0.0 # Initialize
    for i in range(current_idx, num_points - 1):
        segment_dist = calculate_distance(points[i], points[i+1])
        if i + 2 < num_points:
            angle = calculate_angle(points[i], points[i+1], points[i+2])
            if angle > max_corner_angle: # Found a corner at points[i+1]
                dist_to_next_corner = path_dist_to_look_ahead + segment_dist # Distance *to* the corner apex/center
                break
        path_dist_to_look_ahead += segment_dist
    
    # Find distance from last significant corner
    dist_from_last_corner = float('inf')
    path_dist_backward = 0.0 # Initialize
    for i in range(current_idx, 0, -1):
        segment_dist = calculate_distance(points[i], points[i-1])
        if i - 2 >= 0:
            angle = calculate_angle(points[i-2], points[i-1], points[i])
            if angle > max_corner_angle: # Found a corner at points[i-1]
                dist_from_last_corner = path_dist_backward + segment_dist # Distance *from* the corner apex/center
                break
        path_dist_backward += segment_dist

    # Base offset for this point based on local angle (for a subtle mid-corner boost)
    deviation_angle_local = 0
    if current_idx > 0 and current_idx + 1 < num_points:
        deviation_angle_local = calculate_angle(points[current_idx-1], p_curr, points[current_idx+1])
    
    # Calculate target corner offset (0 to 1 based on sharpness)
    target_corner_offset_factor = min(1.0, deviation_angle_local / max_corner_angle)

    # --- Blending Logic ---
    # We want to blend from `offset_factor_straight` to `offset_factor_straight + offset_factor_corner * target_corner_offset_factor`
    # when entering and exiting a corner zone.

    # Approaching a corner
    if dist_to_next_corner < blend_distance_offset:
        blend_progress = 1 - (dist_to_next_corner / blend_distance_offset) # 0 at start of blend zone, 1 at corner
        return offset_factor_straight + (offset_factor_corner * target_corner_offset_factor) * blend_progress
    
    # Exiting a corner
    if dist_from_last_corner < blend_distance_offset:
        blend_progress = 1 - (dist_from_last_corner / blend_distance_offset) # 0 at corner exit, 1 after blend zone
        return offset_factor_straight + (offset_factor_corner * target_corner_offset_factor) * blend_progress
        
    # If the point itself is significantly in a corner (and not in a blend zone)
    if deviation_angle_local > max_corner_angle:
        return offset_factor_straight + offset_factor_corner * target_corner_offset_factor

    # Default to straight offset if not in a corner or blend zone
    return offset_factor_straight


def generate_car_data_with_dynamic_speed_and_deviation(
    csv_filepath, car_name, 
    straight_speed, corner_speed, 
    braking_distance, acceleration_distance, 
    max_corner_angle,
    offset_factor_straight=0.0, offset_factor_corner=0.0,
    offset_blend_distance=0.0 # New parameter for smooth offset
):
    """
    Generates a car's coordinates and timing with dynamic speed profiles and optional path deviation.

    Args:
        csv_filepath (str): Path to the input CSV file.
        car_name (str): Name for the JavaScript constant (e.g., "carA" or "carB").
        straight_speed (float): Top speed on straights.
        corner_speed (float): Speed to maintain within corners.
        braking_distance (float): How many units before a corner to start braking.
        acceleration_distance (float): How many units after a corner to accelerate back to straight speed.
        max_corner_angle (float): Angle threshold for a "sharp" corner (degrees).
        offset_factor_straight (float): Max deviation distance in "straight" sections.
        offset_factor_corner (float): Max deviation distance in "corner" sections.
        offset_blend_distance (float): Distance over which offset smoothly transitions.

    Returns:
        tuple: (JavaScript array string, total_time) or (None, None) if error occurs.
    """
    points_raw = []
    try:
        with open(csv_filepath, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader, None) # <--- ADDED: Skip the header row
            for row in reader:
                if len(row) == 2:
                    try:
                        x = float(row[0].strip())
                        y = float(row[1].strip())
                        points_raw.append({'x': x, 'y': y})
                    except ValueError:
                        print(f"Skipping invalid row: {row}. Make sure x and y are numbers.")
                else:
                    print(f"Skipping malformed row: {row}. Each row should have exactly two values (x,y).")
    except FileNotFoundError:
        print(f"Error: CSV file not found at '{csv_filepath}'")
        return None, None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None, None

    if not points_raw:
        return f"const {car_name}Coordinates = [];", 0.0

    car_coordinates_with_time = []
    current_time = 0.0
    num_points = len(points_raw)

    for i in range(num_points):
        p_curr_original = points_raw[i]
        p_prev_original = points_raw[i-1] if i > 0 else None
        p_next_original_for_offset = points_raw[i+1] if i + 1 < num_points else None

        # Determine turn direction for deviation
        turn_direction = get_turn_direction_for_offset(p_prev_original, p_curr_original, p_next_original_for_offset)

        # Calculate offset magnitude using the new smoothing function
        offset_magnitude = get_smoothed_offset_magnitude(
            points_raw, i, num_points, 
            offset_factor_straight, offset_factor_corner, 
            max_corner_angle, offset_blend_distance
        )

        # Deviate the point IF offset factors are greater than 0 AND there's a turn direction
        deviated_x, deviated_y = p_curr_original['x'], p_curr_original['y'] # Default to original
        
        # Only apply directional offset if it's a clear turn
        if turn_direction != 0 and offset_magnitude > 0.0001: 
            deviated_x, deviated_y = offset_point_smooth_directional(
                p_curr_original, p_next_original_for_offset, # Use current to next for direction
                offset_magnitude, turn_direction
            )
        # Apply straight offset if straight (and offset is desired)
        elif turn_direction == 0 and offset_magnitude > 0.0001: 
            if p_next_original_for_offset: # Ensure there's a next point for direction
                dx = p_next_original_for_offset['x'] - p_curr_original['x']
                dy = p_next_original_for_offset['y'] - p_curr_original['y']
                length = math.sqrt(dx**2 + dy**2)
                if length > 0:
                    unit_dx = dx / length
                    unit_dy = dy / length
                    # Push consistently to the "right" for straights (using unit_dy, -unit_dx)
                    deviated_x = p_curr_original['x'] + unit_dy * offset_magnitude
                    deviated_y = p_curr_original['y'] - unit_dx * offset_magnitude


        # Speed calculation for the segment *leading to* the current point
        time_taken_for_segment = 0
        if i > 0:
            # Use the *actual rendered path* for distance calculation
            segment_distance = calculate_distance(
                {'x': car_coordinates_with_time[-1]['x'], 'y': car_coordinates_with_time[-1]['y']},
                {'x': deviated_x, 'y': deviated_y}
            )
            
            # Speed is determined based on angles of the *original* line
            # as that's the "ideal" line for braking points etc.
            effective_speed = get_effective_speed(
                points_raw, i-1, num_points, # Pass i-1 as the start of the *previous* segment
                straight_speed, corner_speed, 
                braking_distance, acceleration_distance, 
                max_corner_angle
            )
            
            if effective_speed <= 0.1: # Ensure minimum speed
                effective_speed = 0.1

            time_taken_for_segment = segment_distance / effective_speed
            current_time += time_taken_for_segment

        car_coordinates_with_time.append({
            'x': round(deviated_x, 6), # Round for cleaner output
            'y': round(deviated_y, 6),
            'time': round(current_time, 3)
        })

    js_coords = []
    for coord in car_coordinates_with_time:
        js_coords.append(f"{{ x: {coord['x']}, y: {coord['y']}, time: {coord['time']} }}")

    js_array_string = f"const {car_name}Coordinates = [\n\t" + ",\n\t".join(js_coords) + "\n];"
    return js_array_string, current_time

if __name__ == "__main__":
    input_csv_file = "racing_line.csv" # Your CSV file

    # --- Car A: Early Braker / Optimal Line (Reference) ---
    CAR_A_STRAIGHT_SPEED = 50.0       # Top speed on straights
    CAR_A_CORNER_SPEED = 15.0         # Speed in corners (lower than straight)
    CAR_A_BRAKING_DISTANCE = 10.0     # Units before a corner to start braking
    CAR_A_ACCELERATION_DISTANCE = 8.0 # Units after a corner to accelerate back to top speed
    CAR_A_MAX_CORNER_ANGLE = 45       # Angle to consider a "corner"

    # Car A has no deviation, so its offset factors are 0
    CAR_A_OFFSET_STRAIGHT = 0.0
    CAR_A_OFFSET_CORNER = 0.0
    CAR_A_OFFSET_BLEND_DISTANCE = 0.0 # No blend needed as no offset

    # --- Car B: Late Braker / Aggressive (STARTING POINT FOR TUNING) ---
    # These parameters are a starting point for you to tune.
    # They aim to make Car B slightly slower overall with visible deviation.
    CAR_B_STRAIGHT_SPEED = 49.0       # Slightly lower than Car A to compensate for longer path
    CAR_B_CORNER_SPEED = 14.0         # Slightly lower than Car A to compensate for longer path
    CAR_B_BRAKING_DISTANCE = 7.0      # Significantly later braking than Car A
    CAR_B_ACCELERATION_DISTANCE = 7.0 # Slightly quicker acceleration out of corners than A
    CAR_B_MAX_CORNER_ANGLE = 40       # React to slightly less severe corners for braking/accel

    # --- Deviation Parameters for Car B (Tune these for visual path difference) ---
    # These values are increased to make the path deviation more obvious.
    CAR_B_OFFSET_STRAIGHT = 0.5   # Deviation on straights (e.g., 0.5 units to the side)
    CAR_B_OFFSET_CORNER = 4.0     # Max deviation in sharp corners (e.g., 4.0 units wider in corner)
    CAR_B_OFFSET_BLEND_DISTANCE = 10.0 # <--- NEW: Distance over which offset smooths (e.g., 10 units)

    # --- Generate Car A Data ---
    print("\n--- Generating Car A (Early Braker) Data ---")
    car_a_js_output, car_a_total_time = generate_car_data_with_dynamic_speed_and_deviation(
        input_csv_file,
        "carA",
        CAR_A_STRAIGHT_SPEED, CAR_A_CORNER_SPEED,
        CAR_A_BRAKING_DISTANCE, CAR_A_ACCELERATION_DISTANCE,
        CAR_A_MAX_CORNER_ANGLE,
        CAR_A_OFFSET_STRAIGHT, CAR_A_OFFSET_CORNER,
        CAR_A_OFFSET_BLEND_DISTANCE
    )

    if car_a_js_output:
        car_a_output_js_file = "carA_coordinates.js"
        try:
            with open(car_a_output_js_file, 'w') as f:
                f.write(car_a_js_output)
            print(f"Car A Data saved to '{car_a_output_js_file}'")
            print(f"Car A Total Time: {car_a_total_time:.3f} seconds")
        except IOError as e:
            print(f"Could not save Car A data: {e}")

    # --- Generate Car B Data ---
    print("\n--- Generating Car B (Late Braker with Deviated Path) Data ---")
    car_b_js_output, car_b_total_time = generate_car_data_with_dynamic_speed_and_deviation(
        input_csv_file,
        "carB",
        CAR_B_STRAIGHT_SPEED, CAR_B_CORNER_SPEED,
        CAR_B_BRAKING_DISTANCE, CAR_B_ACCELERATION_DISTANCE,
        CAR_B_MAX_CORNER_ANGLE,
        CAR_B_OFFSET_STRAIGHT, CAR_B_OFFSET_CORNER,
        CAR_B_OFFSET_BLEND_DISTANCE # New parameter passed
    )

    if car_b_js_output:
        car_b_output_js_file = "carB_coordinates.js"
        try:
            with open(car_b_output_js_file, 'w') as f:
                f.write(car_b_js_output)
            print(f"Car B Data saved to '{car_b_output_js_file}'")
            print(f"Car B Total Time: {car_b_total_time:.3f} seconds")
        except IOError as e:
            print(f"Could not save Car B data: {e}")

    # --- Compare Results ---
    if car_a_total_time is not None and car_b_total_time is not None:
        print("\n--- Race Summary ---")
        print(f"Car A Final Time: {car_a_total_time:.3f}s")
        print(f"Car B Final Time: {car_b_total_time:.3f}s")
        
        time_diff = car_b_total_time - car_a_total_time
        print(f"Car B is {abs(time_diff):.3f} seconds {'behind' if time_diff > 0 else 'ahead'} Car A.")

        if abs(time_diff - 2.0) < 0.5: # Check if it's roughly 2 second difference
            print("\n*** SUCCESS! Time difference is approximately 2 seconds, as requested! ***")
        else:
            print("\n*** IMPORTANT TUNING GUIDE (for both time and path) ***")
            print("The time difference is NOT yet ~2 seconds. You need to adjust the parameters for CAR_B_.")
            print("The path deviation for Car B will naturally make it slower due to a longer distance, so speed parameters will need compensation.")

            print("\n**HOW TO TUNE CAR B FOR ~2s GAP AND DESIRED DYNAMIC & PATH:**")
            print("1.  **Run the script as is.** Note the initial `Car A Final Time` and `Car B Final Time`.")
            print("2.  **Calculate the current `time_diff = Car B Time - Car A Time`.**")
            ("    * **If `time_diff` is much larger than 2s (e.g., 10s, 15s): Car B is too slow overall.**")
            ("        * **Primary for Time:** INCREASE `CAR_B_STRAIGHT_SPEED` and `CAR_B_CORNER_SPEED` (e.g., in steps of 0.5 to 1.0).")
            ("        * **For Late Braking:** Ensure `CAR_B_BRAKING_DISTANCE` is significantly SMALLER than `CAR_A_BRAKING_DISTANCE`.")
            ("    * **If `time_diff` is negative (Car B wins) or much smaller than 2s: Car B is too fast overall.**")
            ("        * **Primary for Time:** DECREASE `CAR_B_STRAIGHT_SPEED` and `CAR_B_CORNER_SPEED`.")
            print("\n**To adjust path deviation (visuals - run and visualize in your tool):**")
            print("-   **`CAR_B_OFFSET_STRAIGHT`:** Controls deviation on straight sections. Increase for more general side-to-side movement. (e.g., 0.5 to 1.0)")
            print("-   **`CAR_B_OFFSET_CORNER`:** Controls how much wider Car B goes in corners. INCREASE this significantly to make turns visibly different. (e.g., from 3.0 to 4.0 or 5.0)")
            print("-   **`CAR_B_OFFSET_BLEND_DISTANCE`:** This is key for smoothness. INCREASE this value (e.g., from 10.0 to 15.0 or 20.0) to make the transition into and out of corners smoother. Too high might make corners less wide or blend too far.")
            print("\n**Iterate:** Make small changes (especially to speeds and corner offset) and re-run the script. Visualize the paths and check the time difference. This is a highly iterative tuning process.")
