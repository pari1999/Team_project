import cv2
from robomaster import robot
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def start_stream():
    logger.info("[DJI] Starting EP camera stream")

    with robot.Robot() as ep_robot:
        ep_robot.initialize(conn_type="sta", sn="192.168.0.110")  # Replace with IP
        ep_camera = ep_robot.camera

        ep_camera.start_video_stream(display=False)
        for i in range(200):  # Stream 200 frames
            img = ep_camera.read_cv2_image()
            cv2.imshow("RoboMaster EP Camera", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        ep_camera.stop_video_stream()
        cv2.destroyAllWindows()
