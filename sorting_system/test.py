import cv2
import numpy as np
import time
from robomaster import robot

# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
MARKER_ID = 1
MARKER_LENGTH = 0.15  # Marker size in meters

# Approximate camera intrinsics for 640x480 resolution
camera_matrix = np.array([[600, 0, 320],
                          [0, 600, 240],
                          [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))


def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera

    ep_camera.start_video_stream(display=False)
    print("[INFO] Starting video stream and marker tracking...")

    try:
        while True:
            frame = ep_camera.read_cv2_image(strategy="newest", timeout=5)
            if frame is None:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco_detector.detectMarkers(gray)

            if ids is not None and MARKER_ID in ids:
                idx = list(ids.flatten()).index(MARKER_ID)

                # Draw marker boundary
                cv2.aruco.drawDetectedMarkers(frame, corners)

                # Estimate pose
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, MARKER_LENGTH, camera_matrix, dist_coeffs)

                # Draw axis
                #cv2.aruco.utils.drawAxis(frame, camera_matrix, dist_coeffs, rvecs[idx], tvecs[idx], 0.1)


                # Get distance and alignment info
                distance = tvecs[idx][0][2]
                marker_x = corners[idx][0][:, 0].mean()
                frame_center = frame.shape[1] // 2
                offset = marker_x - frame_center

                # Display info
                cv2.putText(frame, f"ID: {MARKER_ID}  Dist: {distance:.2f}m",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                print(f"[INFO] Distance: {distance:.2f} m | Offset: {offset:.2f}")

                if abs(offset) > 20:
                    if offset > 0:
                        ep_chassis.drive_speed(x=0, y=0, z=-10)
                    else:
                        ep_chassis.drive_speed(x=0, y=0, z=10)
                else:
                    if distance > 0.3:
                        ep_chassis.drive_speed(x=0.1, y=0, z=0)
                    else:
                        ep_chassis.drive_speed(x=0, y=0, z=0)
                        print("[INFO] Reached ~30 cm from marker. Stopping.")
                        break
            else:
                ep_chassis.drive_speed(x=0.1, y=0, z=0)
                cv2.putText(frame, "Searching for ArUco Marker...",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Show the robot’s camera view
            cv2.imshow("RoboMaster EP View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.05)

    finally:
        ep_chassis.drive_speed(x=0, y=0, z=0)
        ep_camera.stop_video_stream()
        ep_robot.close()
        cv2.destroyAllWindows()
        print("[DONE] Shutdown complete.")


if __name__ == "__main__":
    main()
