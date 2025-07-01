import cv2
import numpy as np
import robomaster
from robomaster import robot
import time

# Load camera calibration data
calib_data = np.load('sorting_system/dji/ep_camera_calibration.npz')
camera_matrix = calib_data['camera_matrix']
dist_coeffs = calib_data['dist_coeffs']

M = np.array([
    [-18.8522366,  -1.07293859,  28.03411281],
    [ -5.56457726, -2.32509031, 110.2241043 ]
])

# --- Marker size in mm (set to your actual marker size) ---
ARUCO_MARKER_SIZE = 40  # e.g., 40mm

# Initialize the RoboMaster EP
ep_robot = robot.Robot()
ep_robot.initialize(conn_type='ap')  # Connect in AP mode

# Start the camera
ep_camera = ep_robot.camera
ep_camera.start_video_stream(display=False)

# Use the 4x4_50 dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# Initialize the gripper and arm
ep_gripper = ep_robot.gripper
ep_arm = ep_robot.robotic_arm

print("Press 'q' to quit.")

def camera_to_robot(X, Y):
    cam_point = np.array([X, Y, 1])
    robot_point = M @ cam_point
    return robot_point[0], robot_point[1]

try:
    pickup_attempts = 0  # Track number of pickup attempts for marker ID 2
    running = True
    while running:
        # Get a frame from the camera
        frame = ep_camera.read_cv2_image(strategy='newest', timeout=2.5)
        
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == 2:
                    # Draw a bounding box around the marker
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    print("Detected ArUco marker with ID 2!")

                    # Estimate pose of the marker (returns rvec, tvec)
                    # tvec is the translation of the marker center in camera frame (in mm)
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, ARUCO_MARKER_SIZE, camera_matrix, dist_coeffs)
                    if tvecs is not None:
                        # Use the first detected marker (should be only one with ID 2)
                        tvec = tvecs[i][0]  # (X, Y, Z) in camera frame (mm)
                        X_cam, Y_cam = tvec[0], tvec[1]
                        print(f"Marker center in camera frame: X={X_cam:.2f} mm, Y={Y_cam:.2f} mm")
                        # Convert to robot coordinates
                        x_robot, y_robot = camera_to_robot(X_cam, Y_cam)
                        print(f"Moving arm to robot coordinates: x={x_robot:.2f} mm, y={y_robot:.2f} mm")
                        # Open gripper
                        time.sleep(2)
                        ep_gripper.open(power=50)
                        time.sleep(1)
                        ep_gripper.pause()
                        # Move the robotic arm to pick up the object
                        ep_arm.move(x_robot, y_robot)
                        # Wait until the arm reaches the target position
                        time.sleep(2)
                        # Close the gripper to pick up the object
                        ep_gripper.close(power=50)
                        time.sleep(1)
                        ep_gripper.pause()
                        pickup_attempts += 1  # Increment after every pickup attempt
                        if pickup_attempts >= 4:
                            print("Maximum pickup attempts reached. Stopping.")
                            running = False
                            y_above = y_robot + 50  # Move 50mm up
                            print(f"Camera coordinates in range. Moving arm above to x={x_robot:.2f} mm, y={y_above:.2f} mm")
                            ep_arm.move(x_robot, y_above)
                            time.sleep(2)
                            break

        cv2.imshow('DJI RoboMaster ArUco Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the camera and close the connection
    ep_camera.stop_video_stream()
    ep_robot.close()
    cv2.destroyAllWindows() 