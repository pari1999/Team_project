from robomaster import robot , camera
from dji_camera import start_new_stream
from dji_movement import robomaster_move
import time
import logging
import threading


def run():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type = "ap")
    ep_camera = ep_robot.camera

    t1 = threading.Thread(target=robomaster_move,args=(ep_robot,))
    t2 = threading.Thread(target=start_new_stream, args = (ep_camera,))

    t1.start()
    t2.start()
    

if __name__ == "__main__":
    run()
