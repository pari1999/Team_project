from robomaster import robot
from dji import dji_camera
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def run():
    logger.info("[DJI] Starting RoboMaster operations")

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    try:
        # Only run camera logic now
        dji_camera.start_stream(ep_robot)

    except Exception as e:
        logger.error(f"[DJI] Error during RoboMaster operation: {e}")

    finally:
        ep_robot.close()
        logger.info("[DJI] RoboMaster connection closed")
