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
PICKUP_MARKER_ID = 1
DROPOFF_MARKER_ID = 2
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
DOCKING_DISTANCE_M     = 0.30    # stop within 30 cm

SCAN_Z_SPEED = 20.0
YAW_THRESHOLD_DEG = 5.0
LATERAL_THRESHOLD_PX = 50
MAX_LATERAL_SPEED_MS = 0.05
FORWARD_SPEED_MS = 0.10
MIN_ROTATION_SPEED_DPS = 5.0
MARKER_SIZE_M = MARKER_LENGTH  # just to keep naming consistent

def normalize_yaw(yaw_deg):
    # wrap to [-180,180]
    yaw = (yaw_deg + 180) % 360 - 180
    # fold into [-90,90]
    if   yaw >  90: yaw -= 180
    elif yaw < -90: yaw += 180
    return yaw

def automated_forward_pickup(chassis, cam):
    print("[INFO] starting pickup process...")
    mode = "scan"
    scan_dir = 1

    try:
        while True:
            frame = cam.read_cv2_image(strategy="newest", timeout=5)
            if frame is None:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco_detector.detectMarkers(gray)

            vx = vy = wz = 0.0

            # SCAN MODE: spin until marker appears
            if mode == "scan":

                wz = scan_dir * SCAN_Z_SPEED
                cv2.putText(frame, "SCANNING...", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                if ids is not None and PICKUP_MARKER_ID in ids:
                    mode = "approach"
                    print("[INFO] Marker detected → switching to APPROACH")

            # APPROACH MODE
            if mode == "approach":
                if ids is None or PICKUP_MARKER_ID not in ids:
                    mode = "scan"
                    continue

                idx = list(ids.flatten()).index(PICKUP_MARKER_ID)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, MARKER_SIZE_M, camera_matrix, dist_coeffs)
                tz = tvecs[idx][0][2]
                px_mean = corners[idx][0][:, 0].mean()
                cx = frame.shape[1] / 2
                offset_x = px_mean - cx

                R, _ = cv2.Rodrigues(rvecs[idx])
                raw_yaw = np.degrees(np.arctan2(R[0,2], R[2,2]))
                yaw_deg = normalize_yaw(raw_yaw)

                if abs(yaw_deg) > YAW_THRESHOLD_DEG:
                    vx = 0.0
                    vy = 0.0
                    sign = np.sign(yaw_deg)
                    wz = sign * max(MIN_ROTATION_SPEED_DPS,
                                    abs(ROTATION_GAIN * yaw_deg))
                elif abs(offset_x) > LATERAL_THRESHOLD_PX:
                    vx = 0.0
                    wz = 0.0
                    vy = np.clip(0.002 * offset_x,
                                 -MAX_LATERAL_SPEED_MS,
                                  MAX_LATERAL_SPEED_MS)
                else:
                    vy = 0.0
                    wz = 0.0
                    vx = FORWARD_SPEED_MS

                cv2.putText(frame,
                    f"D={tz:.2f}m Off={offset_x:.0f}px Yaw={yaw_deg:.1f}° vx={vx:.2f}",
                    (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                if tz <= DOCKING_DISTANCE_M:
                    print("[INFO] Docked successfully!")
                    break

            chassis.drive_speed(x=vx, y=vy, z=wz)

            cv2.imshow("Docking View", frame)
            if cv2.waitKey(1) == ord('q'):
                break
            time.sleep(0.05)

    finally:
        chassis.drive_speed(x=0, y=0, z=0)
        cam.stop_video_stream()
        cv2.destroyAllWindows()
        print("[DONE]")


def rotate_in_place(ep_chassis):
    print("[INFO] rotating 180 degrees in place...")

    rotation_speed_deg = 60  # deg/sec (adjust if needed)
    rotation_duration_sec = 180 / rotation_speed_deg  # time = angle / speed

    ep_chassis.drive_speed(x=0, y=0, z=rotation_speed_deg)  # z is yaw rate in deg/sec
    time.sleep(rotation_duration_sec)
    ep_chassis.drive_speed(x=0, y=0, z=0)

    print("[INFO] ✅ rotation complete.")


def automated_return_dropoff(chassis, cam):
    print("[INFO] starting dropping process...")
    mode = "scan"
    scan_dir = 1
    cam.start_video_stream(display=True)

    try:
        while True:
            frame = cam.read_cv2_image(strategy="newest", timeout=5)
            if frame is None:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco_detector.detectMarkers(gray)

            vx = vy = wz = 0.0

            # SCAN MODE: spin until marker appears
            if mode == "scan":
                wz = scan_dir * SCAN_Z_SPEED
                cv2.putText(frame, "SCANNING...", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                if ids is not None and DROPOFF_MARKER_ID in ids:
                    mode = "approach"
                    print("[INFO] Marker detected → switching to APPROACH")

            # APPROACH MODE
            if mode == "approach":
                if ids is None or DROPOFF_MARKER_ID not in ids:
                    mode = "scan"
                    continue

                idx = list(ids.flatten()).index(DROPOFF_MARKER_ID)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, MARKER_SIZE_M, camera_matrix, dist_coeffs)
                tz = tvecs[idx][0][2]
                px_mean = corners[idx][0][:, 0].mean()
                cx = frame.shape[1] / 2
                offset_x = px_mean - cx

                R, _ = cv2.Rodrigues(rvecs[idx])
                raw_yaw = np.degrees(np.arctan2(R[0,2], R[2,2]))
                yaw_deg = normalize_yaw(raw_yaw)

                if abs(yaw_deg) > YAW_THRESHOLD_DEG:
                    vx = 0.0
                    vy = 0.0
                    sign = np.sign(yaw_deg)
                    wz = sign * max(MIN_ROTATION_SPEED_DPS,
                                    abs(ROTATION_GAIN * yaw_deg))
                elif abs(offset_x) > LATERAL_THRESHOLD_PX:
                    vx = 0.0
                    wz = 0.0
                    vy = np.clip(0.002 * offset_x,
                                 -MAX_LATERAL_SPEED_MS,
                                  MAX_LATERAL_SPEED_MS)
                else:
                    vy = 0.0
                    wz = 0.0
                    vx = FORWARD_SPEED_MS

                cv2.putText(frame,
                    f"D={tz:.2f}m Off={offset_x:.0f}px Yaw={yaw_deg:.1f}° vx={vx:.2f}",
                    (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

                if tz <= DOCKING_DISTANCE_M:
                    print("[INFO] Docked successfully!")
                    break

            chassis.drive_speed(x=vx, y=vy, z=wz)

            cv2.imshow("Docking View", frame)
            if cv2.waitKey(1) == ord('q'):
                break
            time.sleep(0.05)

    finally:
        chassis.drive_speed(x=0, y=0, z=0)
        cam.stop_video_stream()
        cv2.destroyAllWindows()
        print("[DONE]")