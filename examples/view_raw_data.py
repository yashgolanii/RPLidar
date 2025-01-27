from rplidar import RPLidar

LIDAR_PORT = '/dev/ttyUSB0'  # Adjust for your setup

def view_raw_data():
    lidar = RPLidar(LIDAR_PORT)
    try:
        print("Starting LiDAR...")
        for scan in lidar.iter_measures():  # or iter_scans() for processed scans
            print(scan)  # Print raw tuples
    except KeyboardInterrupt:
        print("Stopping LiDAR...")
    finally:
        lidar.stop()
        lidar.disconnect()

if __name__ == "__main__":
    view_raw_data()
