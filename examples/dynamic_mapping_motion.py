import math
import time
import serial
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


def polar_to_cartesian(angle_deg, distance_mm):
    """Convert polar coordinates to Cartesian."""
    angle_rad = math.radians(angle_deg)
    x = distance_mm * math.cos(angle_rad)
    y = distance_mm * math.sin(angle_rad)
    return x, y


def get_tilt_angle():
    """Calculate tilt angle by integrating gyroscope data."""
    tilt_angle = 0
    last_time = time.time()

    while True:
        try:
            # Read a line from the IMU
            line = imu_serial.readline().decode('utf-8').strip()
            if line.startswith("GY:"):  # Check for the prefix "GY:"
                gz_raw = float(line.replace("GY:", "").strip())  # Extract numeric part
                gz = gz_raw # Convert raw value to degrees/sec (scale factor for MPU9250)

                # Compute time interval
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                # Integrate to get tilt angle
                tilt_angle += gz * dt
                return tilt_angle
            else:
                print(f"Ignoring non-gyroscope data: {line}")
        except ValueError as ve:
            print(f"Error reading tilt angle: {ve} (raw data: {line})")
            return 0.0
        except Exception as e:
            print(f"Unexpected error: {e}")
            return 0.0



def transform_to_3d(local_points, tilt_angle):
    """Transform 2D LIDAR points to 3D based on tilt angle."""
    global_points = []
    tilt_rad = math.radians(tilt_angle)
    for x, y in local_points:
        z = y * math.sin(tilt_rad)
        y_proj = y * math.cos(tilt_rad)
        global_points.append((x, y_proj, z))
    return global_points


def update_map(lidar, ax, scatter):
    global global_map
    for scan in lidar.iter_scans():
        local_points = []
        for _, angle, distance in scan:
            if distance > 0 and distance <= DMAX:
                x, y = polar_to_cartesian(angle, distance)
                local_points.append((x, y))

        tilt_angle = get_tilt_angle()  # Get current tilt angle
        global_points = transform_to_3d(local_points, tilt_angle)
        global_map.extend(global_points)

        if global_map:
            x_coords = [p[0] for p in global_map]
            y_coords = [p[1] for p in global_map]
            z_coords = [p[2] for p in global_map]
            scatter._offsets3d = (x_coords, y_coords, z_coords)
        lidar.clean_input()
        plt.pause(0.01)


def main():
    try:
        print("Starting LIDAR and IMU...")
        print(lidar.get_info)
        print(lidar.express_data)
        print(lidar.motor_running)
        print(lidar.motor_speed)
        lidar.stop()
        lidar.disconnect()
        lidar.connect()
        lidar.clean_input()
        lidar.start_motor()
        lidar.motor_speed=60
        

        imu_serial.reset_input_buffer()

        # Set up 3D plot
        plt.ion()
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("3D Mapping with Tilted LIDAR")
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
