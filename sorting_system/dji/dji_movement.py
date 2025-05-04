from robomaster import robot
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def move_straight():
    with robot.Robot() as ep_robot:
        ep_robot.initialize(conn_type="sta", sn="192.168.0.110")  # Replace with IP
        ep_robot.chassis.move(x=1.0, y=0, z=0, xy_speed=0.5).wait_for_completed()
