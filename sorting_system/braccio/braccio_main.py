from color_detection import ColorObjectDetector
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    logger.info("Starting Braccio Color Sorting System...")
    detector = ColorObjectDetector()
    detector.run() 

if __name__ == "__main__":
    main()
