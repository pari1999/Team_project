# dji_movement.py
import cv2
import numpy as np
from robomaster import robot, sensor
import logging
import time
from dji import tof_sensor

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
MARKER_ID = 1
MARKER_LENGTH = 0.15  # in meters

# Load camera calibration
data = np.load("ep_camera_calibration.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]

# Constants
YAW_THRESHOLD = 1.0        # degrees
DOCKING_DISTANCE = 0.3     # meters
ROTATION_GAIN = -0.05
ROTATION_MIN = 1.0         # deg/sec

def automated_forward_pickup(ep_chassis, ep_camera):
    print("[INFO] starting pickup process...")

    last_seen_yaw_deg = 0.0
    lost_yaw_direction = 1

    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=5)
        if frame is None:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detector.detectMarkers(gray)

        forward_speed = 0.0
        lateral_speed = 0.0
        rotation_speed = 0.0

        if ids is not None and MARKER_ID in ids:
            idx = list(ids.flatten()).index(MARKER_ID)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_LENGTH, camera_matrix, dist_coeffs)

            distance = tvecs[idx][0][2]
            marker_x = corners[idx][0][:, 0].mean()
            frame_center = frame.shape[1] // 2
            offset_x = marker_x - frame_center

            # Lateral alignment
            lateral_speed = 0.002 * offset_x
            lateral_speed = np.clip(lateral_speed, -0.05, 0.05)

            # Yaw estimation
            rotation_matrix, _ = cv2.Rodrigues(rvecs[idx])
            yaw_rad = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            yaw_deg = np.degrees(yaw_rad)

            # Optional: amplify yaw if far
            if distance > 0.8:
                yaw_deg *= (1 + 0.5 * (distance - 0.8))

            last_seen_yaw_deg = yaw_deg
            lost_yaw_direction = -np.sign(yaw_deg) if yaw_deg != 0 else 1

            # Yaw correction
            if abs(yaw_deg) > YAW_THRESHOLD:
                rotation_speed = ROTATION_GAIN * yaw_deg
                if abs(rotation_speed) < ROTATION_MIN:
                    rotation_speed = np.sign(rotation_speed) * ROTATION_MIN
                forward_speed = 0.0
            else:
                rotation_speed = 0.0
                forward_speed = 0.1

            # Final docking condition
            if distance <= DOCKING_DISTANCE and abs(yaw_deg) <= YAW_THRESHOLD:
                print("[INFO] ✅ reached successfully at 30cm and aligned.")
                break

            print(f"[DEBUG] Distance: {distance:.2f}m | Offset: {offset_x:.1f}px | Yaw: {yaw_deg:.2f}°")

            cv2.putText(frame, f"Dist: {distance:.2f}m | Offset: {offset_x:.1f}px | Yaw: {yaw_deg:.1f}°",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        else:
            # Marker lost – rotate in last known direction
            forward_speed = 0.0
            lateral_speed = 0.0
            rotation_speed = lost_yaw_direction * ROTATION_MIN
            print("[WARN] Marker lost. Scanning...")

            cv2.putText(frame, "Searching for ArUco Marker...",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        ep_chassis.drive_speed(x=forward_speed, y=lateral_speed, z=rotation_speed)

        cv2.imshow("robot View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.05)

    ep_chassis.drive_speed(x=0, y=0, z=0)
    print("[INFO] reached the location and stopped.")



