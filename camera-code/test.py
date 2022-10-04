import cv2
import time
from picamera2 import Picamera2

# Grab images as numpy arrays and leave everything else to OpenCV.
cv2.startWindowThread()

picam2 = Picamera2()
config = picam2.create_still_configuration(main={"format": 'XRGB8888', "size": (1920, 1080)})
picam2.configure(config)
picam2.start()


# img = cv2.imread(cv2.samples.findFile("/home/pi/Pictures/2022-09-15 14:13:44.177047/img10.png"))
# cv2.imshow("img", img)

time.sleep(2)

while True:
    picam2.capture_file("test.png")
    im = picam2.capture_array()
    time.sleep(1)
    cv2.imwrite("test2.png",im)
    cv2.imshow("Camera", im)
    picam2.set_controls({"AfTrigger": 0})
    time.sleep(10)

