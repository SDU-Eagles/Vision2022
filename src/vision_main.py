from multiprocessing import Process
import time

from drone_control import DroneControl
from marker_processing import marker_processing


def drone_processing():
    DC = DroneControl()
    DC.print_debug('Brief pause before continuing....')
    time.sleep(1.0)
    DC.run()


if __name__ == '__main__':
    process_marker_processing = Process(target=marker_processing)
    process_marker_processing.start()
    process_marker_processing.join()
    
    process_drone_processing = Process(target=drone_processing)
    process_drone_processing.start()
    process_drone_processing.join()
    
