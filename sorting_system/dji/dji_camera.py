import cv2
import logging
import time
import numpy as np
import torch
from pathlib import Path
from robomaster import robot

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

MODEL_PATH = str(Path(__file__).parent.parent / "models" / "yolov5n.pt")

def detect_and_pick():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper

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

            # Draw detections
            for i, row in detections.iterrows():
                if row['name'] != 'bottle':
                    continue
                x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, 'bottle', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("RoboMaster EP Camera", img)
            cv2.waitKey(1)

            # If detected, pick it up at fixed position
            for _, row in detections.iterrows():
                if row['name'] == 'bottle' and row['confidence'] > 0.5:
                    logger.info("[DJI] Bottle detected, picking up at fixed position")
                    ep_arm.move(130, -30).wait_for_completed()
                    time.sleep(0.5)
                    ep_gripper.close()
                    time.sleep(1)
                    for y in range(-30, 61, 10):  # from -30 to 60 in steps of 10
                        ep_arm.move(130, y).wait_for_completed()
                        time.sleep(0.2)
                    logger.info("[DJI] Bottle picked up")
                    ep_camera.stop_video_stream()
                    cv2.destroyAllWindows()
                    ep_robot.close()
                    return
    finally:
        ep_camera.stop_video_stream()
        cv2.destroyAllWindows()
        ep_robot.close()

if __name__ == "__main__":
    detect_and_pick()