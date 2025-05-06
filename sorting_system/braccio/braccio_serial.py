import serial
import time
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def send_command(cmd):
    logger.info(f"[Braccio] Sending: {cmd}")
    with serial.Serial('COM3', 9600, timeout=1) as ser:
        time.sleep(2)
        ser.write((cmd + "\\n").encode())
