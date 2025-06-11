from robomaster import robot, sensor
from dji import dji_movement
from dji import dji_robot_initialize
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def run():
    logger.info("[DJI] Starting RoboMaster operations")

    try:
        dji_movement.move_in_square_safely(dji_robot_initialize.ep_robot)  # ✅ Now it should run
    except Exception as e:
        logger.error(f"[DJI] Error: {e}")
  
