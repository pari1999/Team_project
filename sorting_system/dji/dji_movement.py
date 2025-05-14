import logging
import time

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def drive_pattern(ep_robot):
    logger.info("[DJI] Drive pattern skipped (not used in this mode)")

def arm_up_down(ep_robot):
    logger.info("[DJI] Arm movement skipped (handled in camera logic)")
