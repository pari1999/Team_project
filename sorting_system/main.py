import threading
from dji import dji_main
from braccio import braccio_main
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

if __name__ == "__main__":
    logger.info("Starting sorting system")
    t1 = threading.Thread(target=dji_main.run)
    #t2 = threading.Thread(target=braccio_main.run)
    t1.start()
    #t2.start()
    #t1.join()

    
   # t2.join()
