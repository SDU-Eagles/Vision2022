import cv2
import time
from picamera2 import Picamera2
from PIL import Image
import piexif

# Grab images as numpy arrays and leave everything else to OpenCV.
cv2.startWindowThread()

picam2 = Picamera2()
config = picam2.create_still_configuration(main={"format": 'XRGB8888',"size": (4056,3040)})
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
    cv2.line(im,(10,10),(100,100),(255,0,255))
    cv2.imwrite("test3.png",im)
    # cv2.imshow("Camera", im)
    img = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    im_pil = Image.fromarray(img)
    gps_ifd = {
        piexif.GPSIFD.GPSAltitude: (51,1),
        piexif.GPSIFD.GPSLatitude: ((25,1),( 13,1),( 48.343,1)),
        piexif.GPSIFD.GPSLongitude: ((25,1),( 13,1),( 48.343,1) )
    }
    exif_dict = {"GPS": gps_ifd}
    exif_bytes = piexif.dump(exif_dict)
    im_pil.save("test5.jpg","jpeg",exif=exif_bytes)
    picam2.set_controls({"AfTrigger": 0})
    time.sleep(10)