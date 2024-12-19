import math
import matplotlib.pyplot as plt
from rplidar import RPLidar

# LIDAR setup
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)

# Parameters
DMAX = 4000  # Maximum distance in mm to include points
POINT_SIZE = 1  # Size of points in the plot
TRANS_STEP = 50  # Translational step in mm per frame

# Global map and position
global_map = []  # List to store all points in the global frame
current_position = [0, 0]  # LIDAR's position in the global frame (x, y)

# Functions
def polar_to_cartesian(angle_deg, distance_mm):
    """Convert polar coordinates to Cartesian coordinates."""
    angle_rad = math.radians(angle_deg)
    x = distance_mm * math.cos(angle_rad)
    y = distance_mm * math.sin(angle_rad)
    return x, y

def transform_to_global(local_points, lidar_position):
    """Transform points from the LIDAR's local frame to the global frame."""
    global_points = []
    for x, y in local_points:
        global_x = x + lidar_position[0]
        global_y = y + lidar_position[1]
        global_points.append((global_x, global_y))
    return global_points

def update_map(lidar, ax, scatter):
    """Fetch data from LIDAR, transform, and update the map."""
    global current_position, global_map
    for scan in lidar.iter_scans():
        # Collect points in the local frame
        local_points = []
        for _, angle, distance in scan:
            if distance > 0 and distance <= DMAX:  # Filter out invalid or out-of-range points
                x, y = polar_to_cartesian(angle, distance)
                local_points.append((x, y))
        
        # Transform to global frame
        global_points = transform_to_global(local_points, current_position)
        global_map.extend(global_points)

        # Simulate translational motion
        current_position[0] += TRANS_STEP  # Move LIDAR along x-axis for simplicity

        # Update scatter plot
        if global_map:
            x_coords = [p[0] for p in global_map]
            y_coords = [p[1] for p in global_map]
            scatter.set_offsets(list(zip(x_coords, y_coords)))
            plt.pause(0.01)  # Adjust for smoother animation

# Main execution
def main():
    try:
        print("Starting LIDAR...")
        lidar.connect()
        
        # Set up the plot
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_title("Dynamic Mapping with Translational Motion")
        ax.set_xlim(-DMAX * 2, DMAX * 2)
        ax.set_ylim(-DMAX * 2, DMAX * 2)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.grid(True)
        scatter = ax.scatter([], [], s=POINT_SIZE)

        # Dynamic mapping
        update_map(lidar, ax, scatter)

    except KeyboardInterrupt:
        print("Stopping LIDAR...")

    finally:
        lidar.stop()
        lidar.disconnect()
        print("LIDAR stopped.")

if __name__ == "__main__":
    main()