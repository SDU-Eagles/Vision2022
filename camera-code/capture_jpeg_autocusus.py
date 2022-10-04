#!/usr/bin/python3
import time
from datetime import datetime
import os
from picamera2 import Picamera2

picam2 = Picamera2()
config = picam2.create_still_configuration(main={"format": 'XRGB8888', "size": (1920, 1080)})
picam2.configure(config)
picam2.set_controls({"AfMode": 2})
print(picam2.controls)
picam2.start()


path = "/home/pi/Pictures/" + str(datetime.now())

os.mkdir(path)
time.sleep(2)

for x in range(100000):
    picam2.capture_file( path + "/img" + str(x) + ".png")
    picam2.set_controls({"AfTrigger": 0})
    time.sleep(1)

picam2.close()
