import sqlite3
import matplotlib.pyplot as plt

# Function to initialize the SQLite database
def init_db():
    print("Initializing database connection...")
    conn = sqlite3.connect('explored_coords.db')
    c = conn.cursor()
    
    # Create a table to store GPS coordinates
    c.execute('''CREATE TABLE IF NOT EXISTS coordinates (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    x REAL,
                    y REAL,
                    z REAL)''')
    conn.commit()
    conn.close()
    print("Database initialized and table created (if not already present).")

# Function to insert coordinates into the database
def store_coordinates(x, y, z):
    print(f"Inserting coordinates: x={x}, y={y}, z={z}")
    conn = sqlite3.connect('explored_coords.db')
    c = conn.cursor()
    
    # Insert the new coordinates into the database
    c.execute('''INSERT INTO coordinates (x, y, z) VALUES (?, ?, ?)''', (x, y, z))
    conn.commit()
    conn.close()
    print("Coordinates inserted into the database.")

# Function to get all coordinates
def get_all_coordinates():
    print("Fetching all coordinates from the database...")
    conn = sqlite3.connect('explored_coords.db')
    c = conn.cursor()
    c.execute('SELECT x, y, z FROM coordinates')
    coordinates = c.fetchall()  # Returns a list of tuples
    conn.close()
    print(f"Retrieved {len(coordinates)} coordinates from the database.")
    return coordinates

# Function to plot the coordinates on a 2D map
def plot_coordinates(coordinates):
    print("Plotting coordinates...")
    x_coords = [coord[0] for coord in coordinates]
    y_coords = [coord[1] for coord in coordinates]
    plt.scatter(x_coords, y_coords, color='blue', s=1)
    plt.title('Explored Coordinates')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.show()

# Initialize database (if not already initialized)
init_db()

# Retrieve all stored coordinates
coordinates = get_all_coordinates()

# Plot the coordinates
plot_coordinates(coordinates)
