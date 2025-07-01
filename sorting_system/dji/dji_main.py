from robomaster import robot, sensor
from dji import dji_movement, dji_camera
from dji import dji_robot_initialize
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def run():
    try:
        dji_movement.automated_forward_pickup(dji_robot_initialize.ep_chassis,dji_robot_initialize.ep_camera)  # ✅ Now it should run
        dji_camera.main()
        dji_movement.rotate_in_place(dji_robot_initialize.ep_chassis)
        dji_movement.automated_return_dropoff(dji_robot_initialize.ep_chassis,dji_robot_initialize.ep_camera)
        dji_movement.rotate_in_place(dji_robot_initialize.ep_chassis)
    except Exception as e:
        logger.error(f"[DJI] Error: {e}")
  
