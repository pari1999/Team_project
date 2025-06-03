from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'  # Adjust for your system
MIN_HUMAN_DIST = 800  # mm

def detect_obstacle():
    lidar = RPLidar(PORT_NAME)
    try:
        for scan in lidar.iter_scans(max_buf_meas=500):
            for (_, angle, distance) in scan:
                # Check 0° to 60° and 300° to 360° (front range)
                if (angle <= 60 or angle >= 300) and distance < MIN_HUMAN_DIST:
                    print(f"[LIDAR] Obstacle at {int(distance)} mm, angle {int(angle)}°")
                    return True
            return False
    finally:
        lidar.stop()
        lidar.disconnect()
