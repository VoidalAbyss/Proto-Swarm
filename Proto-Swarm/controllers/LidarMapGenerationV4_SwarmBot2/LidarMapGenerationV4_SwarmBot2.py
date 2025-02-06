import sqlite3
import numpy as np
import matplotlib.pyplot as plt
from controller import Robot
import time

# Constants
TIME_STEP = 64
MAX_SPEED = 6.28
GRID_RESOLUTION = 0.05       # Size of each grid cell (meters)
LIDAR_RANGE_THRESHOLD = 0.7  # Distance threshold to avoid obstacles (meters)
DOOR_WIDTH_THRESHOLD = 0.8   # Width threshold for detecting doors (meters)
TURNING_SPEED_FACTOR = 0.3   # Reduced speed for smoother turning
MIN_TURN_ANGLE = 15          # Minimum angle to turn when too close to a wall
STUCK_TIME_THRESHOLD = 5     # Time in seconds before rotating when stuck
RETURN_TIME_THRESHOLD = 15   # Time in seconds to return to original position
REVERSE_DURATION = 2
DATABASE = 'D:\Webot\Database\explored_coords.db'

# Initial map vertices (corners of the map)
VERTICES = [
    [-1.33, 0.83, 0.08164907307970759],
    [1.33, 0.83, 0.08164907307970759],
    [-1.33, -0.83, 0.08164907307970759],
    [1.33, -0.83, 0.08164900832711564]
]

# Initialize robot and sensors
robot = Robot()
lidar = robot.getDevice("lidar_custom")
gps = robot.getDevice("gps_custom")
left_motor = robot.getDevice("motor_1")
right_motor = robot.getDevice("motor_2")

# Activate sensors
lidar.enable(TIME_STEP)
lidar.enablePointCloud()
gps.enable(TIME_STEP)
left_motor.setPosition(float('inf'))  # Set motor for velocity control
right_motor.setPosition(float('inf'))

def clear_database():
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()
    conn.commit()
    conn.close()

# Function to initialize the SQLite database
def init_db(vertices, resolution):
    x_min = min([v[0] for v in vertices])
    x_max = max([v[0] for v in vertices])
    y_min = min([v[1] for v in vertices])
    y_max = max([v[1] for v in vertices])

    conn = sqlite3.connect(DATABASE, timeout=10.0)
    try:
        c = conn.cursor()
        c.execute('''
            CREATE TABLE IF NOT EXISTS coordinates (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x REAL,
                y REAL,
                z REAL,
                visited INTEGER DEFAULT 0
            )
        ''')
        z_value = vertices[0][2]
        for x in np.arange(x_min, x_max + resolution, resolution):
            for y in np.arange(y_min, y_max + resolution, resolution):
                c.execute('INSERT OR IGNORE INTO coordinates (x, y, z, visited) VALUES (?, ?, ?, ?)',
                          (x, y, z_value, 0))
        conn.commit()
    finally:
        conn.close()


# Function to update the visited status of a coordinate
def update_visited(x, y, z):
    conn = sqlite3.connect(DATABASE, timeout=10.0)
    try:
        c = conn.cursor()
        c.execute('''
            UPDATE coordinates 
            SET visited = 1 
            WHERE ABS(x - ?) < 0.1 AND ABS(y - ?) < 0.1 AND ABS(z - ?) < 0.1
        ''', (x, y, z))
        conn.commit()
    finally:
        conn.close()


# Function to get all coordinates and visited status
def get_all_coordinates():
    conn = sqlite3.connect(DATABASE)
    c = conn.cursor()
    c.execute('SELECT x, y, z, visited FROM coordinates')
    coordinates = c.fetchall()  # Returns a list of tuples (x, y, z, visited)
    conn.close()
    return coordinates

# Function to generate a grid and mark the visited locations
def generate_map(coordinates, resolution=0.1):
    print("Generate map functions called");
    # Extract the x, y coordinates and visited status
    x_coords = [coord[0] for coord in coordinates]
    y_coords = [coord[1] for coord in coordinates]
    visited = [coord[3] for coord in coordinates]

    # Find the min and max coordinates for the grid bounds
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    # Define the grid size based on resolution
    x_bins = int((x_max - x_min) / resolution) + 1
    y_bins = int((y_max - y_min) / resolution) + 1

    # Create a grid to mark visited locations
    grid = np.zeros((y_bins, x_bins), dtype=int)

    # Mark the visited positions in the grid
    for x, y, v in zip(x_coords, y_coords, visited):
        # Map the coordinates to grid indices
        x_idx = int((x - x_min) / resolution)
        y_idx = int((y - y_min) / resolution)

        if 0 <= x_idx < x_bins and 0 <= y_idx < y_bins:
            grid[y_idx, x_idx] = v  # Mark the grid cell based on visited status

    return grid, x_min, y_min, resolution

def lidar_to_world_coordinates(lidar_data, robot_position, resolution):
    """
    Converts LIDAR data into world coordinates of traversable points.
    """
    lidar_angles = np.linspace(-np.pi / 2, np.pi / 2, len(lidar_data))  # Adjust angles based on LIDAR specs
    traversable_points = []

    for distance, angle in zip(lidar_data, lidar_angles):
        if distance < LIDAR_RANGE_THRESHOLD:
            # Convert polar coordinates to Cartesian
            dx = distance * np.cos(angle)
            dy = distance * np.sin(angle)

            # Transform to world coordinates
            x_world = robot_position[0] + dx
            y_world = robot_position[1] + dy

            # Snap to grid resolution
            x_snapped = round(x_world / resolution) * resolution
            y_snapped = round(y_world / resolution) * resolution

            traversable_points.append((x_snapped, y_snapped))

    return traversable_points

def update_traversable_areas(lidar_data, gps_position):
    traversable_points = lidar_to_world_coordinates(lidar_data, gps_position, GRID_RESOLUTION)
    conn = sqlite3.connect(DATABASE, timeout=10.0)
    try:
        c = conn.cursor()
        for x, y in traversable_points:
            c.execute('''
                UPDATE coordinates 
                SET visited = 1 
                WHERE ABS(x - ?) < 0.1 AND ABS(y - ?) < 0.1
            ''', (x, y))
        conn.commit()
    finally:
        conn.close()

# Function to plot the map live
def plot_map_live():
    print("Plot_map_live Called");
    # Retrieve all coordinates from the database
    coordinates = get_all_coordinates()

    if not coordinates:
        print("No Coordinates found");
        return

    # Generate the map based on visited coordinates
    grid, x_min, y_min, resolution = generate_map(coordinates, resolution=GRID_RESOLUTION)

    plt.clf()  # Clear the previous map
    plt.imshow(grid, cmap='Greens', origin='lower', interpolation='nearest', vmin=0, vmax=1)
    plt.title("Robot's Real-time Explored Area")
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.colorbar(label='Visited/Unvisited')
    plt.draw()
    plt.pause(0.1)  # Pause to update the plot live

def lidar_to_world_coordinates_obstacles(lidar_data, robot_position):
    """
    Converts LIDAR data into world coordinates of detected points.
    """
    lidar_angles = np.linspace(-np.pi / 2, np.pi / 2, len(lidar_data))  # Adjust angles based on LIDAR specs
    detected_points = []

    for distance, angle in zip(lidar_data, lidar_angles):
        if distance < LIDAR_RANGE_THRESHOLD:  # Only consider points within the LIDAR range threshold
            # Convert polar coordinates to Cartesian
            dx = distance * np.cos(angle)
            dy = distance * np.sin(angle)

            # Transform to world coordinates
            x_world = robot_position[0] + dx
            y_world = robot_position[1] + dy

            detected_points.append((x_world, y_world))

    return detected_points

def plot_lidar_points(lidar_data, gps_position):
    """
    Extracts LIDAR points and overlays them on the map plot.
    """
    detected_points = lidar_to_world_coordinates_obstacles(lidar_data, gps_position)
    x_points = [point[0] for point in detected_points]
    y_points = [point[1] for point in detected_points]

    # Plot the LIDAR-detected points
    plt.scatter(x_points, y_points, c='red', s=10, label="Detected Objects/Walls", alpha=0.7)
    plt.legend()



# Helper functions
def reverse():
    # Reverse the robot slightly to get unstuck
    left_motor.setVelocity(-MAX_SPEED * 0.5)
    right_motor.setVelocity(-MAX_SPEED * 0.5)
    robot.step(REVERSE_DURATION * 1000 // TIME_STEP)  # Reverse for a short duration
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def rotate_left_90():
    # Rotate the robot left by 90 degrees
    left_motor.setVelocity(-MAX_SPEED * TURNING_SPEED_FACTOR)
    right_motor.setVelocity(MAX_SPEED * TURNING_SPEED_FACTOR)
    steps_for_90_deg_turn = int(2 * 1000 / TIME_STEP)
    for _ in range(steps_for_90_deg_turn):
        robot.step(TIME_STEP)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    robot.step(TIME_STEP)

def avoid_obstacle(lidar_data):
    # Obstacle avoidance using LIDAR
    front_distance = lidar_data[len(lidar_data) // 2]
    left_side_distance = lidar_data[0]
    right_side_distance = lidar_data[-1]

    if front_distance < LIDAR_RANGE_THRESHOLD:
        if left_side_distance > right_side_distance:
            left_motor.setVelocity(-MAX_SPEED * 0.5)
            right_motor.setVelocity(MAX_SPEED * 0.5)
        else:
            left_motor.setVelocity(MAX_SPEED * 0.5)
            right_motor.setVelocity(-MAX_SPEED * 0.5)
    else:
        left_motor.setVelocity(MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)

def mark_visited(grid, gps_position, x_min, y_min, resolution):
    # Mark the current position on the grid as visited
    x_idx = int((gps_position[0] - x_min) / resolution)
    y_idx = int((gps_position[1] - y_min) / resolution)
    if 0 <= x_idx < len(grid[0]) and 0 <= y_idx < len(grid):
        grid[y_idx][x_idx] = 1  # Mark as visited

def create_grid(vertices, resolution):
    x_min = min([v[0] for v in vertices])
    x_max = max([v[0] for v in vertices])
    y_min = min([v[1] for v in vertices])
    y_max = max([v[1] for v in vertices])
    width = int((x_max - x_min) / resolution)
    height = int((y_max - y_min) / resolution)
    return np.zeros((height, width), dtype=int), x_min, y_min, resolution

def calculate_coverage(grid):
    total_cells = grid.size
    visited_cells = np.sum(grid == 1)
    return visited_cells / total_cells

# Function to explore the environment and update the map live
def explore():
    init_db(VERTICES, GRID_RESOLUTION)  # Initialize database if not already initialized
    
    grid, x_min, y_min, resolution = create_grid(VERTICES, GRID_RESOLUTION)
    original_position = gps.getValues()
    stuck_start_time = None
    last_position = original_position

    plt.ion()  # Interactive mode for real-time plotting
    plt.figure(figsize=(8, 8))

    while robot.step(TIME_STEP) != -1:
        gps_position = gps.getValues()
        lidar_data = lidar.getRangeImage()
        mark_visited(grid, gps_position, x_min, y_min, resolution)
        
        update_traversable_areas(lidar_data, gps_position)
        
        x, y, z = gps_position[0], gps_position[1], gps_position[2]

        # Store the current coordinates in the database if not already visited
        update_visited(x, y, z)

        # Update the map live
        plot_map_live()
        
        plot_lidar_points(lidar_data, gps_position)
        plt.pause(0.1)  # Pause to update the plot
        
        # Simulate robot movement (you can add obstacle avoidance, etc. here)
        #left_motor.setVelocity(MAX_SPEED)
        #right_motor.setVelocity(MAX_SPEED)

        distance_moved = np.linalg.norm(np.array(gps_position[:2]) - np.array(last_position[:2]))

        if distance_moved < 0.05:
            if stuck_start_time is None:
                stuck_start_time = time.time()
            stuck_duration = time.time() - stuck_start_time
            if stuck_duration > STUCK_TIME_THRESHOLD:
                reverse()
                rotate_left_90()
                stuck_start_time = None
            elif stuck_duration > RETURN_TIME_THRESHOLD:
                stuck_start_time = None
        else:
            stuck_start_time = None
            last_position = gps_position

        avoid_obstacle(lidar_data)

        coverage = calculate_coverage(grid)
        print(f"BOT_2: Current map coverage: {coverage * 100:.2f}%")

        if coverage >= 0.8:
            print("BOT_2: Exploration complete.")
            plt.ioff()  # Turn off interactive mode
            plt.show()
            break
        
        time.sleep(TIME_STEP / 1000)  # Wait for the next step

# Start the exploration
clear_database() # Call the function to clear the database before starting the simulation
plt.ion()  # Enable interactive mode for live plotting
explore()
