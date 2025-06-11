from robomaster import robot, sensor
from dji import dji_movement
from dji import dji_robot_initialize
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def run():
    try:
        dji_movement.automated_forward_pickup(dji_robot_initialize.ep_chassis,dji_robot_initialize.ep_camera)  # ✅ Now it should run
    except Exception as e:
        logger.error(f"[DJI] Error: {e}")
  
