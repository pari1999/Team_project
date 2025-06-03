import cv2
import logging
import time
import numpy as np
import torch
from pathlib import Path
import argparse
from robomaster import robot

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

MODEL_PATH = str(Path(__file__).parent.parent / "models" / "yolov5n.pt")

def calculate_optimal_approach(frame_height, box_height, box_width):
    """
    Calculate if the robot is at optimal distance for pickup based on Object size
    relative to frame and aspect ratio.
    """
    frame_coverage = box_height / frame_height
    aspect_ratio = box_height / box_width if box_width > 0 else 0
    
    # Ideal bottle should take up about 45-55% of frame height
    # and maintain expected bottle aspect ratio (typically 2.5-3.5)
    ideal_coverage = (0.45, 0.55)
    ideal_aspect_ratio = (2.5, 3.5)
    
    is_optimal = (ideal_coverage[0] <= frame_coverage <= ideal_coverage[1] and 
                 ideal_aspect_ratio[0] <= aspect_ratio <= ideal_aspect_ratio[1])
    
    return is_optimal, frame_coverage

def start_stream(ep_robot):
    logger.info("[DJI] Starting YOLOv5 detection and pick-up routine")

    ep_camera = ep_robot.camera
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_chassis = ep_robot.chassis

    cv2.namedWindow("RoboMaster EP Camera", cv2.WINDOW_NORMAL)
    ep_camera.start_video_stream(display=True)
    ep_gripper.open()

    model = torch.hub.load('ultralytics/yolov5', 'custom', path=MODEL_PATH, force_reload=False)
    model.conf = 0.5

    try:
        while True:
            img = ep_camera.read_cv2_image()
            if img is None:
                continue
            results = model(img)
            detections = results.pandas().xyxy[0]
            logger.info(f"Detections: {detections}")

            for i, row in detections.iterrows():
                x1, y1, x2, y2, conf, cls, label = (
                    int(row['xmin']), int(row['ymin']),
                    int(row['xmax']), int(row['ymax']),
                    row['confidence'], row['class'], row['name']
                )
                # Shrink bounding box by 20% around center
                box_w = x2 - x1
                box_h = y2 - y1
                shrink_factor = 0.2
                new_w = int(box_w * (1 - shrink_factor))
                new_h = int(box_h * (1 - shrink_factor))
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                new_x1 = max(center_x - new_w // 2, 0)
                new_y1 = max(center_y - new_h // 2, 0)
                new_x2 = min(center_x + new_w // 2, img.shape[1] - 1)
                new_y2 = min(center_y + new_h // 2, img.shape[0] - 1)
                if conf > 0.5:
                    cv2.rectangle(img, (new_x1, new_y1), (new_x2, new_y2), (0, 0, 255), 4)
                    cv2.putText(img, str(label), (new_x1, new_y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            cv2.imshow("RoboMaster EP Camera", img)
            cv2.waitKey(1)

            if not detections.empty:
                row = detections.iloc[0]
                x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                label = row['name']
                if label != "bottle":
                    continue

                object_center_x = (x1 + x2) // 2
                frame_center_x = img.shape[1] // 2
                logger.info(f"[DJI] Object detected: {label}")

                # Align horizontally
                while abs(object_center_x - frame_center_x) > 30:
                    ep_chassis.drive_speed(x=0, y=0, z=10 if object_center_x < frame_center_x else -10)
                    time.sleep(0.2)
                    ep_chassis.drive_speed(x=0, y=0, z=0)
                    img = ep_camera.read_cv2_image()
                    if img is None:
                        break
                    results = model(img)
                    detections = results.pandas().xyxy[0]
                    if detections.empty:
                        break
                    row = detections.iloc[0]
                    x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                    object_center_x = (x1 + x2) // 2

                # Approach object (move until optimal distance)
                box_measurements = []
                while True:
                    img = ep_camera.read_cv2_image()
                    if img is None:
                        break
                    results = model(img)
                    detections = results.pandas().xyxy[0]
                    if detections.empty:
                        break
                    row = detections.iloc[0]
                    x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                    box_height = y2 - y1
                    box_width = x2 - x1
                    object_center_x = (x1 + x2) // 2
                    
                    # Keep track of recent measurements
                    box_measurements.append((box_height, box_width))
                    if len(box_measurements) > 4:
                        box_measurements.pop(0)
                    
                    # Use average measurements to reduce noise
                    avg_height = sum(m[0] for m in box_measurements) / len(box_measurements)
                    avg_width = sum(m[1] for m in box_measurements) / len(box_measurements)
                    
                    is_optimal, coverage = calculate_optimal_approach(img.shape[0], avg_height, avg_width)
                    
                    logger.info(f"[Approach] Current coverage: {coverage:.2f}, Height: {avg_height:.1f}px, Width: {avg_width:.1f}px")

                    if is_optimal:
                        logger.info(f"[Approach] Reached optimal position (coverage: {coverage:.2f}). Doing final alignment.")
                        # Final alignment: center horizontally
                        for _ in range(3):
                            img = ep_camera.read_cv2_image()
                            if img is None:
                                break
                            results = model(img)
                            detections = results.pandas().xyxy[0]
                            if detections.empty:
                                break
                            row = detections.iloc[0]
                            x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                            object_center_x = (x1 + x2) // 2
                            frame_center_x = img.shape[1] // 2
                            if abs(object_center_x - frame_center_x) > 10:
                                ep_chassis.drive_speed(x=0, y=0, z=10 if object_center_x < frame_center_x else -10)
                                time.sleep(0.2)
                                ep_chassis.drive_speed(x=0, y=0, z=0)
                            else:
                                break
                        # Optionally move forward a little more
                        ep_chassis.drive_speed(x=0.05, y=0, z=0)
                        time.sleep(0.3)
                        ep_chassis.drive_speed(x=0, y=0, z=0)
                        break

                    # Move forward with speed based on how far we are
                    if avg_height < 400:  # Adjusted threshold
                        speed, step_time = 0.2, 0.4
                    elif avg_height < 500:  # Adjusted threshold
                        speed, step_time = 0.12, 0.3
                    else:
                        speed, step_time = 0.07, 0.2

                    ep_chassis.drive_speed(x=speed, y=0, z=0)
                    time.sleep(step_time)
                    ep_chassis.drive_speed(x=0, y=0, z=0)

                # Pick up object
                box_height = y2 - y1
                box_width = x2 - x1
                forward_distance, down_angle, final_angle = calculate_gripper_positions(box_height, box_width)
                
                # Open gripper wider for larger bottles
                gripper_width = min(1.0, 0.5 + (box_width / 640))  # Scale gripper width based on bottle width
                ep_gripper.open(gripper_width)
                
                # Move arm to calculated positions
                ep_arm.move(forward_distance, down_angle).wait_for_completed()
                ep_arm.move(1, down_angle - 20).wait_for_completed()  # Move slightly more down
                time.sleep(0.5)
                ep_gripper.close()
                time.sleep(1)
                ep_arm.move(1, final_angle).wait_for_completed()
                logger.info(f"[DJI] Object picked with adaptive positions - height: {box_height}px, width: {box_width}px")
                break
    finally:
        ep_camera.stop_video_stream()
        cv2.destroyAllWindows()

def calculate_gripper_positions(box_height, box_width):
    """Calculate appropriate gripper positions based on bottle dimensions."""
    # Convert pixel dimensions to relative units (assuming 640x480 camera resolution)
    height_ratio = box_height / 480
    width_ratio = box_width / 640
    
    # Calculate gripper positions
    # Base positions with adjustments based on bottle size
    forward_distance = 0.5 + (height_ratio * 0.3)  # Adjust forward distance based on height
    down_angle = -20 - (height_ratio * 30)  # Adjust down angle based on height
    final_angle = 30 + (height_ratio * 20)  # Adjust final angle based on height
    
    return forward_distance, down_angle, final_angle

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibrate', action='store_true', help='Run calibration mode for approach distance')
    args = parser.parse_args()
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    try:
        start_stream(ep_robot)
    finally:
        ep_robot.close()

if __name__ == "__main__":
    main()
