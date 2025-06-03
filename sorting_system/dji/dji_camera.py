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

TARGET_BOTTLE_HEIGHT = 200 # to change as per the video

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

                # Approach object (move until bottle appears small enough)
                box_height_history = []
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
                    object_center_x = (x1 + x2) // 2
                    box_height_history.append(box_height)
                    if len(box_height_history) > 4:
                        box_height_history.pop(0)
                    avg_box_height = sum(box_height_history) / len(box_height_history)

                    logger.info(f"[Approach] Avg box height: {avg_box_height:.1f}px (target: {TARGET_BOTTLE_HEIGHT}px)")

                    # Allow a buffer zone
                    if avg_box_height <= TARGET_BOTTLE_HEIGHT + 20:  # Increased buffer zone
                        logger.info(f"[Approach] Bottle is close enough (avg height: {avg_box_height:.1f}px). Doing final alignment.")
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
                    if avg_box_height < 400:  # Adjusted threshold
                        speed, step_time = 0.2, 0.4
                    elif avg_box_height < 500:  # Adjusted threshold
                        speed, step_time = 0.12, 0.3
                    else:
                        speed, step_time = 0.07, 0.2

                    ep_chassis.drive_speed(x=speed, y=0, z=0)
                    time.sleep(step_time)
                    ep_chassis.drive_speed(x=0, y=0, z=0)

                # Pick up object
                ep_arm.move(0.5, -20).wait_for_completed()
                ep_arm.move(1, -40).wait_for_completed()
                time.sleep(0.5)
                ep_gripper.close()
                time.sleep(1)
                ep_arm.move(1, 30).wait_for_completed()
                logger.info("[DJI] Object picked")
                break
    finally:
        ep_camera.stop_video_stream()
        cv2.destroyAllWindows()

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
