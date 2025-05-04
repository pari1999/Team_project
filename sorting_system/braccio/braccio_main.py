from braccio import braccio_camera, braccio_object, braccio_serial
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def run():
    logger.info("[Braccio] Starting Braccio object sorter")
    frame = braccio_camera.get_frame()
    if braccio_object.object_detected(frame):
        braccio_serial.send_command("PICK_OBJECT")
        time.sleep(2)
