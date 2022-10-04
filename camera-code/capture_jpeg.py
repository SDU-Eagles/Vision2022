#!/usr/bin/python3
import time
from datetime import datetime
import os
from picamera2 import Picamera2

picam2 = Picamera2()
config = picam2.create_still_configuration()
picam2.configure(config)

picam2.start()


path = "/home/pi/Pictures/" + str(datetime.now())

os.mkdir(path)
time.sleep(2)

for x in range(100000):
    picam2.capture_file( path + "/img" + str(x) + ".png")

picam2.close()