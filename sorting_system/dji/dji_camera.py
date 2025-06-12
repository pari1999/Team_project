import cv2
import logging
import time
import numpy as np
import torch
from pathlib import Path
from robomaster import robot
import os

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

MODEL_PATH = str(Path(__file__).parent.parent / "models" / "yolov5n.pt")

def detect_and_pick(centering_threshold=30, min_box_width=120, min_box_height=128):
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_chassis = ep_robot.chassis

    cv2.namedWindow("RoboMaster EP Camera", cv2.WINDOW_NORMAL)
    ep_camera.start_video_stream(display=True)
    ep_gripper.open()

    # Ensure captured_frames directory exists
    frames_dir = os.path.join(os.path.dirname(__file__), 'captured_frame')
    os.makedirs(frames_dir, exist_ok=True)

    try:
        frame_count = 0
        while True:
            img = ep_camera.read_cv2_image()
            if img is None:
                continue

            # Detect blue box using color thresholding
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([100, 150, 50])
            upper_blue = np.array([140, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            largest_area = 0
            largest_bbox = None
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                area = w * h
                if area > largest_area:
                    largest_area = area
                    largest_bbox = (x, y, w, h)

            if largest_bbox is not None and largest_area > 1000:  # filter out small noise
                x, y, w, h = largest_bbox
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(img, 'blue box', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                # Save frame
                frame_path = os.path.join(frames_dir, f"frame_{frame_count}.jpg")
                cv2.imwrite(frame_path, img)
                logger.info(f"[DJI] Blue box detected at (x={x}, y={y}, w={w}, h={h}), saved to {frame_path}")
                frame_count += 1

                # Centering logic
                img_center_x = img.shape[1] // 2
                box_center_x = x + w // 2
                offset_x = box_center_x - img_center_x

                if abs(offset_x) > centering_threshold:
                    # Rotate towards the box
                    rotation_speed = 10 if offset_x > 0 else -10  # right if box is right, left if left
                    ep_chassis.drive_speed(x=0, y=0, z=rotation_speed)
                    logger.info(f"[DJI] Centering blue box: offset_x={offset_x}, rotating {'right' if offset_x > 0 else 'left'}.")
                else:
                    # Stop rotating when centered
                    ep_chassis.drive_speed(x=0, y=0, z=0)
                    logger.info("[DJI] Blue box centered.")
                    # Check if box is big enough
                    if w < min_box_width or h < min_box_height:
                        # Move forward a little to make the box bigger
                        logger.info(f"[DJI] Box too small (w={w}, h={h}), moving forward to increase size.")
                        ep_chassis.drive_speed(x=0.2, y=0, z=0)  # Move forward slowly
                        time.sleep(0.3)  # Move for 0.3 seconds (adjust as needed for 2-4cm)
                        ep_chassis.drive_speed(x=0, y=0, z=0)  # Stop
                        time.sleep(0.2)
                        continue  # Re-detect after moving forward
                    logger.info("[DJI] Blue box big enough, picking up at fixed position")
                    ep_arm.move(130, -30).wait_for_completed()
                    time.sleep(0.5)
                    ep_gripper.close()
                    time.sleep(1)
                    # Move arm above after pick up (higher y value, e.g., 60 to 120)
                    for y_arm in range(-30, 121, 20):  # from -30 to 120 in steps of 20
                        ep_arm.move(130, y_arm).wait_for_completed()
                        time.sleep(0.2)
                    logger.info("[DJI] Blue box picked up")
                    ep_camera.stop_video_stream()
                    cv2.destroyAllWindows()
                    ep_robot.close()
                    return
            else:
                # Rotate in place to search for blue object
                ep_chassis.drive_speed(x=0, y=0, z=10)

            cv2.imshow("RoboMaster EP Camera", img)
            cv2.waitKey(1)
    finally:
        ep_camera.stop_video_stream()
        cv2.destroyAllWindows()
        ep_robot.close()

if __name__ == "__main__":
    import sys
    threshold = 0
    min_w = 120
    min_h = 100
    if len(sys.argv) > 1:
        try:
            threshold = int(sys.argv[1])
        except ValueError:
            threshold = 0
    if len(sys.argv) > 2:
        try:
            min_w = int(sys.argv[2])
        except ValueError:
            min_w = 120
    if len(sys.argv) > 3:
        try:
            min_h = int(sys.argv[3])
        except ValueError:
            min_h = 100
    detect_and_pick(centering_threshold=threshold, min_box_width=min_w, min_box_height=min_h)