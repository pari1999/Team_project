import cv2
import logging
import time
import numpy as np
import torch
from pathlib import Path
from dji import lidar_scan  

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load YOLOv5n model
MODEL_PATH = str(Path(__file__).parent.parent / "models" / "yolov5n.pt")
model = torch.hub.load('ultralytics/yolov5', 'custom', path=MODEL_PATH, force_reload=False)
model.conf = 0.5  # Confidence threshold

def start_stream(ep_robot):
    logger.info("[DJI] Starting YOLOv5 + LIDAR integrated detection")

    ep_camera = ep_robot.camera
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_chassis = ep_robot.chassis

    ep_camera.start_video_stream(display=True)
    ep_gripper.open()

    while True:
        img = ep_camera.read_cv2_image()
        if img is None:
            continue

        results = model(img)
        detections = results.pandas().xyxy[0]

        for i, row in detections.iterrows():
            x1, y1, x2, y2, conf, cls, label = (
                int(row['xmin']), int(row['ymin']),
                int(row['xmax']), int(row['ymax']),
                row['confidence'], row['class'], row['name']
            )

            if conf > 0.5:
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                object_center_x = (x1 + x2) // 2
                frame_center_x = img.shape[1] // 2

                logger.info(f"[DJI] Object detected: {label} ({conf:.2f})")

                user_input = input(f"Object '{label}' detected. Pick up? (y/n): ").strip().lower()
                if user_input == 'y':
                    logger.info("[DJI] Aligning with object")

                    # Rotate to center object
                    while abs(object_center_x - frame_center_x) > 30:
                        if object_center_x < frame_center_x:
                            logger.info("Turning left")
                            ep_chassis.drive_speed(x=0, y=0, z=10)
                        else:
                            logger.info("Turning right")
                            ep_chassis.drive_speed(x=0, y=0, z=-10)

                        time.sleep(0.2)
                        ep_chassis.drive_speed(x=0, y=0, z=0)

                        img = ep_camera.read_cv2_image()
                        if img is None:
                            break
                        results = model(img)
                        detections = results.pandas().xyxy[0]
                        if detections.empty:
                            logger.info("Lost object")
                            break

                        row = detections.iloc[0]
                        x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                        object_center_x = (x1 + x2) // 2

                    # LIDAR check before moving forward
                    logger.info("[DJI] Checking for humans/obstacles with LiDAR")
                    if lidar_scan.detect_obstacle():
                        logger.warning("[DJI] Human or obstacle detected! Movement stopped.")
                        continue

                    # Move forward
                    logger.info("Path clear. Moving forward.")
                    ep_chassis.drive_speed(x=0.2, y=0, z=0)
                    time.sleep(1.5)
                    ep_chassis.drive_speed(x=0, y=0, z=0)

                    # Pick object
                    ep_arm.move(1, -40).wait_for_completed()
                    time.sleep(0.5)
                    ep_gripper.close()
                    time.sleep(1)
                    ep_arm.move(1, 30).wait_for_completed()
                    time.sleep(0.5)

                    logger.info("[DJI] Object picked")
                    break

        cv2.imshow("RoboMaster EP Camera", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    ep_camera.stop_video_stream()
    cv2.destroyAllWindows()
