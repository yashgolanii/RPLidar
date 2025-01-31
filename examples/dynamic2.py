import math
import time
import serial
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rplidar import RPLidar

# Configuration
LIDAR_PORT = '/dev/ttyUSB0'
IMU_PORT = '/dev/ttyACM0'
IMU_BAUDRATE = 9600
DMAX = 4000  # Maximum distance in mm
POINT_SIZE = 1

# Initialize LIDAR and IMU
lidar = RPLidar(LIDAR_PORT)
imu_serial = serial.Serial(IMU_PORT, IMU_BAUDRATE, timeout=1)
global_map = []

# Shared variables for IMU data
tilt_angle = 0.0  # Pitch (X-axis)
yaw_angle = 0.0   # Yaw (Z-axis)
tilt_lock = threading.Lock()

def polar_to_cartesian(angle_deg, distance_mm):
    """Convert polar coordinates to Cartesian."""
    angle_rad = math.radians(angle_deg)
    x = distance_mm * math.cos(angle_rad)
    y = distance_mm * math.sin(angle_rad)
    return x, y

def read_imu_data():
    """Read IMU data in a separate thread."""
    global tilt_angle, yaw_angle
    while True:
        try:
            line = imu_serial.readline().decode('utf-8').strip()
            if line.startswith("GY:"):
                gy_raw = float(line.replace("GY:", "").strip())
                with tilt_lock:  # Safely update shared variable
                    tilt_angle += gy_raw * 0.01  # Example integration for pitch
            if line.startswith("AZ:"):  # Assuming yaw is in a different line (AZ is just an example)
                az_raw = float(line.replace("AZ:", "").strip())
                with tilt_lock:  # Safely update shared variable
                    yaw_angle += az_raw * 0.01  # Example integration for yaw
        except Exception as e:
            print(f"IMU read error: {e}")

def transform_to_3d(local_points):
    """Transform 2D LiDAR points to 3D using the latest tilt and yaw angles."""
    global tilt_angle, yaw_angle
    global_points = []
    with tilt_lock:  # Safely read shared variable
        tilt_rad = math.radians(tilt_angle)
        yaw_rad = math.radians(yaw_angle)
    
    for x, y in local_points:
        # Apply pitch (tilt) first
        z = y * math.sin(tilt_rad)
        y_proj = y * math.cos(tilt_rad)
        
        # Apply yaw (rotation around Z-axis)
        x_rot = x * math.cos(yaw_rad) - y_proj * math.sin(yaw_rad)
        y_rot = x * math.sin(yaw_rad) + y_proj * math.cos(yaw_rad)
        
        global_points.append((x_rot, y_rot, z))
    return global_points

def update_map(lidar, ax, scatter):
    global global_map
    scan_count = 0

    for scan in lidar.iter_scans():
        local_points = []
        for _, angle, distance in scan:
            if distance > 0 and distance <= DMAX:
                x, y = polar_to_cartesian(angle, distance)
                local_points.append((x, y))

        global_points = transform_to_3d(local_points)
        global_map.extend(global_points)

        # Update plot every 10 scans
        scan_count += 1
        if scan_count % 10 == 0:
            # Downsample points for faster rendering
            x_coords = [p[0] for p in global_map]
            y_coords = [p[1] for p in global_map]
            z_coords = [p[2] for p in global_map]
            scatter._offsets3d = (x_coords, y_coords, z_coords)
            plt.pause(0.01)

def main():
    try:
        print("Starting LIDAR and IMU...")
        status, error_code = lidar.get_health()
        print(f"LIDAR health status: {status}, Error code: {error_code}")

        # Start IMU thread
        imu_thread = threading.Thread(target=read_imu_data, daemon=True)
        imu_thread.start()

        # Set up 3D plot
        plt.ion()
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("3D Mapping with Tilted LIDAR (Pitch + Yaw)")
        ax.set_xlim(-DMAX, DMAX)
        ax.set_ylim(-DMAX, DMAX)
        ax.set_zlim(-DMAX, DMAX)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        scatter = ax.scatter([], [], [], s=POINT_SIZE)

        update_map(lidar, ax, scatter)

    except KeyboardInterrupt:
        print("Stopping LIDAR and IMU...")

    finally:
        lidar.stop()
        lidar.disconnect()
        imu_serial.close()
        print("LIDAR and IMU stopped")

if __name__ == "__main__":
    main()
