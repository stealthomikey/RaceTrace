import csv

def csv_to_js_coordinates(csv_filepath):
    """
    Reads a CSV file containing x, y coordinates and converts it into
    a JavaScript array string format.

    Args:
        csv_filepath (str): The path to the input CSV file.

    Returns:
        str: A string representing the JavaScript array of coordinate objects,
             or None if an error occurs.
    """
    coordinates = []
    try:
        with open(csv_filepath, 'r') as csvfile:
            reader = csv.reader(csvfile)
            # You might want to skip the header row if your CSV has one
            # next(reader, None) # Uncomment this line if your CSV has a header row like 'x,y'

            for row in reader:
                if len(row) == 2:
                    try:
                        x = float(row[0].strip())
                        y = float(row[1].strip())
                        coordinates.append(f"{{ x: {x}, y: {y} }}")
                    except ValueError:
                        print(f"Skipping invalid row: {row}. Make sure x and y are numbers.")
                else:
                    print(f"Skipping malformed row: {row}. Each row should have exactly two values (x,y).")
    except FileNotFoundError:
        print(f"Error: CSV file not found at '{csv_filepath}'")
        return None
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return None

    js_array_string = "const outerCoordinates = [\n\t" + ",\n\t".join(coordinates) + "\n];"
    return js_array_string

if __name__ == "__main__":
    # --- IMPORTANT ---
    # Replace 'your_coordinates.csv' with the actual path to your CSV file.
    # Make sure the CSV file is in the same directory as this script, or provide its full path.
    input_csv_file = "outer_perimeter.csv"

    # Example usage:
    js_output = csv_to_js_coordinates(input_csv_file)

    if js_output:
        print("\nGenerated JavaScript Array (copy and paste this into your program):\n")
        print(js_output)

        # Optionally, save the output to a .js file
        output_js_file = "outterCoordinates.js"
        try:
            with open(output_js_file, 'w') as f:
                f.write(js_output)
            print(f"\nAlso saved to '{output_js_file}'")
        except IOError as e:
            print(f"Could not save JavaScript output to file: {e}")
