# dji_movement.py
from robomaster import robot, sensor
import logging
import time
from dji import tof_sensor

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def move_in_square_safely(ep_robot):
    chassis = ep_robot.chassis

    def safe_forward(distance_m=1.0, speed=0.2):
        logger.info(f"[DJI] Moving forward {distance_m} meter(s) (safely)...")
        duration_sec = distance_m / speed
        start_time = time.time()

        while time.time() - start_time < duration_sec:
            if tof_sensor.check_obstacle(chassis):
                logger.warning("[DJI] Obstacle detected! Stopping movement.")
                chassis.drive_speed(x=0, y=0, z=0)
                return False
            chassis.drive_speed(x=speed, y=0, z=0)
            time.sleep(0.1)

        chassis.drive_speed(x=0, y=0, z=0)
        logger.info(f"[DJI] Finished moving forward {distance_m} meter.")
        return True

    def turn_right_90():
        logger.info("[DJI] Turning right 90 degrees...")
        chassis.move(x=0, y=0, z=-90, xy_speed=60).wait_for_completed()
        logger.info("[DJI] Completed 90-degree right turn.")

    # Move in a square pattern
    for i in range(4):
        logger.info(f"[DJI] Side {i+1} of the square")
        if not safe_forward():
            logger.warning(f"[DJI] Aborting square path due to obstacle on side {i+1}.")
            return
        turn_right_90()

    logger.info("[DJI] Completed square path.")



