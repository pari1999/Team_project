import cv2
import logging
import time
import numpy as np
from pathlib import Path
from robomaster import robot
import os

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


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

            # Convert to HSV for color detection
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Define color ranges
            # Red color range (red wraps around in HSV)
            lower_red1 = np.array([0, 150, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 150, 50])
            upper_red2 = np.array([180, 255, 255])
            
            # Green color range
            lower_green = np.array([40, 150, 50])
            upper_green = np.array([80, 255, 255])

            # Commented out Blue detection
            # lower_blue = np.array([100, 150, 50])
            # upper_blue = np.array([140, 255, 255])

            # Create masks for red and green
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            # Combine masks
            mask = cv2.bitwise_or(mask_red, mask_green)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            largest_area = 0
            largest_bbox = None
            detected_color = None

            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                area = w * h
                if area > largest_area:
                    # Check which color is detected
                    roi = hsv[y:y+h, x:x+w]
                    red_pixels = cv2.countNonZero(cv2.bitwise_or(
                        cv2.inRange(roi, lower_red1, upper_red1),
                        cv2.inRange(roi, lower_red2, upper_red2)
                    ))
                    green_pixels = cv2.countNonZero(cv2.inRange(roi, lower_green, upper_green))
                    
                    if red_pixels > green_pixels:
                        detected_color = 'red'
                    else:
                        detected_color = 'green'
                    
                    largest_area = area
                    largest_bbox = (x, y, w, h)

            if largest_bbox is not None and largest_area > 1000:  # filter out small noise
                x, y, w, h = largest_bbox
                color = (0, 0, 255) if detected_color == 'red' else (0, 255, 0)
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, f'{detected_color} object', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Save frame
                frame_path = os.path.join(frames_dir, f"frame_{frame_count}.jpg")
                cv2.imwrite(frame_path, img)
                logger.info(f"[DJI] {detected_color.capitalize()} object detected at (x={x}, y={y}, w={w}, h={h}), saved to {frame_path}")
                frame_count += 1

                # Centering logic
                img_center_x = img.shape[1] // 2
                box_center_x = x + w // 2
                offset_x = box_center_x - img_center_x

                if abs(offset_x) > centering_threshold:
                    # Rotate towards the object
                    rotation_speed = 10 if offset_x > 0 else -10
                    ep_chassis.drive_speed(x=0, y=0, z=rotation_speed)
                    logger.info(f"[DJI] Centering {detected_color} object: offset_x={offset_x}, rotating {'right' if offset_x > 0 else 'left'}.")
                else:
                    # Stop rotating when centered
                    ep_chassis.drive_speed(x=0, y=0, z=0)
                    logger.info(f"[DJI] {detected_color.capitalize()} object centered.")
                    
                    # Check if object is big enough
                    if w < min_box_width or h < min_box_height:
                        logger.info(f"[DJI] Object too small (w={w}, h={h}), moving forward to increase size.")
                        ep_chassis.drive_speed(x=0.2, y=0, z=0)
                        time.sleep(0.3)
                        ep_chassis.drive_speed(x=0, y=0, z=0)
                        time.sleep(0.2)
                        continue

                    logger.info(f"[DJI] {detected_color.capitalize()} object big enough, picking up at fixed position")
                    ep_arm.move(130, -30).wait_for_completed()
                    time.sleep(0.5)
                    ep_gripper.close()
                    time.sleep(1)
                    
                    # Move arm above after pick up
                    for y_arm in range(-30, 121, 20):
                        ep_arm.move(130, y_arm).wait_for_completed()
                        time.sleep(0.2)
                    
                    logger.info(f"[DJI] {detected_color.capitalize()} object picked up")
                    ep_camera.stop_video_stream()
                    cv2.destroyAllWindows()
                    ep_robot.close()
                    return
            else:
                # Rotate in place to search for objects
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
