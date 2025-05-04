from dji import dji_movement, dji_camera
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def run():
    logger.info("[DJI] Starting RoboMaster operations")
    dji_camera.start_stream()
    dji_movement.move_straight()
