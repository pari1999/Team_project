# tof_sensor.py
import logging
from robomaster import robot, sensor
from dji import dji_robot_initialize

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

ep_sensor = sensor.DistanceSensor(dji_robot_initialize.ep_robot)

# Shared variable
value = -1

def set_tof(val):
    global value
    value = val

def get_tof():
    global value
    return value

def handle_tof(data):
    set_tof(data[0])

ep_sensor.sub_distance(freq=50, callback=handle_tof)

def check_obstacle(chassis, min_range=1, max_range=350):
    distance = get_tof()
    logger.info(f"[TOF] Distance: {distance} cm")
    if min_range <= distance <= max_range:
        logger.warning("[TOF] Obstacle detected! Stopping robot.")
        chassis.drive_speed(x=0, y=0, z=0)
        return True
    return False
