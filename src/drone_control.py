#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rpi_capture_image_mavlink.py: Triggering image captures based on RC MAVLink messages 

"""

###############################################
# Definitions                                 #
###############################################
RC_CAPTURE = 6 # RX Channel 7

###############################################
# Standard Imports                            #
###############################################
import time
import threading
import os

###############################################
# MAVlink Imports                             #
###############################################
from pymavlink import mavutil


###############################################
# OpenCV Imports                              #
###############################################

import cv2


###############################################
# RPi Imports                                 #
###############################################
from picamera2 import Picamera2
import datetime



#import RPi.GPIO as GPIO
#servo = 17
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(servo, GPIO.OUT)
#p = GPIO.PWM(servo, 50) #GPIO pin 17 for PWM (50Hz)
#p.start(2.5) #Initialize servo

###############################################
# exif import                                 #
###############################################
from PIL import Image
import piexif
from fractions import Fraction

###############################################
# Drone Control class                         #
###############################################

import numpy as np

import os

class DroneControl:
    def __init__(self, port = "/dev/ttyCubePilot", baud=115200, img_loc = "/home/pi/Pictures") :
        self.ns = "DroneControl"
        self.armed = False
        self.mode = None
        self.rc_channels = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.index = 0
        self.debounce = False
        self.uav_position = [0,0,0,0]
        self.debug = True

        self.image_mod = 0

        self.state = "INIT"
        self.gps_loc_filename='gps_locations.log'
        # Make New folder for photos
        self.folder_loc = (img_loc +'/{}/').format(datetime.datetime.now().strftime('%d-%b_%H.%M'))
        if not os.path.exists(self.folder_loc):
            os.mkdir(self.folder_loc)
        self.print_debug('Saving media to: {}'.format(self.folder_loc))
        
        self.camera = self.init_camera()  # change camera properties within this function#       
        
        # initialise MAVLink Connection
        self.interface = mavutil.mavlink_connection(port, baud=baud, autoreconnect=True)
        self.t_run = threading.Thread(target=self.recv_mavlink)
        self.set_state("RUNNING")
        self.t_run.start()
        self.print_debug(">> MAVlink communication is running (Thread)")
        ## end __init__

    def init_camera(self):
        '''
        Initialise the raspberry pi camera with desired properties. 
        For information on the picamera API, see following URL:
        https://picamera.readthedocs.io/en/release-1.13/api_camera.html#
        '''
        picam2 = Picamera2()
        config = picam2.create_still_configuration(main={"format": 'XRGB8888', "size": (4056,3040)})
        picam2.configure(config)
        picam2.start()
        camera = picam2

        self.fouced_at_alt = 0
        #camera.shutter_speed = 10000 # in microseconds.  1/100 second
        
        return camera

    def print_debug(self,msg):
        ''' Print a message to the console with the prefix "[DroneControl]: '''
        if self.debug == True:
            print("[{}]: >> {} ".format(self.ns,msg))

    def set_state(self, data):
        self.state = data
        self.print_debug("New State: {}".format(data))

    def to_deg(self,value, loc):
        """convert decimal coordinates into degrees, munutes and seconds tuple
        Keyword arguments: value is float gps-value, loc is direction list ["S", "N"] or ["W", "E"]
        return: tuple like (25, 13, 48.343 ,'N')
        """
        if value < 0:
            loc_value = loc[0]
        elif value > 0:
            loc_value = loc[1]
        else:
            loc_value = ""
        abs_value = abs(value)
        deg =  int(abs_value)
        t1 = (abs_value-deg)*60
        min = int(t1)
        sec = round((t1 - min)* 60, 5)
        return (deg, min, sec, loc_value)


    def change_to_rational(self,number):
        """convert a number to rantional
        Keyword arguments: number
        return: tuple like (1, 2), (numerator, denominator)
        """
        f = Fraction(str(number))
        return (f.numerator, f.denominator)
    
    
    def capture(self):
        'captures images and gps locations of said images'
        if not self.debounce:
            img_loc = self.folder_loc+'img_'+str(self.index)
            im = self.camera.capture_array()

            img = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
            im_pil = Image.fromarray(img)
            
            print("lat = " + str(self.uav_position[0]) + " lon = " + str(self.uav_position[1]))

            lat_deg = self.to_deg(self.uav_position[0], ["S", "N"])
            lng_deg = self.to_deg(self.uav_position[1], ["W", "E"])
            exiv_lat = (self.change_to_rational(lat_deg[0]), self.change_to_rational(lat_deg[1]), self.change_to_rational(lat_deg[2]))
            exiv_lng = (self.change_to_rational(lng_deg[0]), self.change_to_rational(lng_deg[1]), self.change_to_rational(lng_deg[2]))
            gps_ifd = {
                piexif.GPSIFD.GPSAltitude: (self.change_to_rational(self.uav_position[2])),
                piexif.GPSIFD.GPSLatitude: (exiv_lat),
                piexif.GPSIFD.GPSLongitude: (exiv_lng ),
                piexif.GPSIFD.GPSHeading: (self.change_to_rational(self.uav_position[3]))
            }
            exif_dict = {"GPS": gps_ifd}
            exif_bytes = piexif.dump(exif_dict)
            im_pil.save(img_loc+'.jpg',"jpeg",exif=exif_bytes)
            msg="Image captured. Saved image as img_{}.jpg".format(self.index)
            self.print_debug(msg)
            if (self.image_mod == 0):
                self.camera.set_controls({"AfTrigger": 0})
                time.sleep(1)
            self.image_mod = (self.image_mod + 1)%50

            # save geolocation of image
#            try:
#                write_drone_location = open(self.folder_loc+self.gps_loc_filename,'a')
                
#                output = '{},{:.7f},{:.7f},{:.2f},{}\n'.format(
#                    self.index,
#                    self.uav_position[0],
#                    self.uav_position[1],
#                    self.uav_position[2],
#                    self.uav_position[3],
#                )

#                write_drone_location.write(output)
#                write_drone_location.close()
#            except TypeError:
#                self.print_debug('WARN: No GPS fix... skipping file save')
#                pass
            self.index += 1
            self.debounce = True

    def recv_mavlink(self):
        while self.state == "RUNNING":
            m = self.interface.recv_msg()
            # if (m != None):
            #     print(m)
            if m:
                #print(m)
                if(m.get_type() == 'HEARTBEAT'):
                    #print(m)
                    self.armed = self.decode_armed(m.base_mode)
                    self.mode = self.decode_custom_flightmode(m.custom_mode)
                elif(m.get_type() == 'RC_CHANNELS'):
                    #self.print_debug(m.chan1_raw)
                    self.rc_channels = [m.chan1_raw, # Throttle
                                     m.chan2_raw, # Roll
                                     m.chan3_raw, # Pitch
                                     m.chan4_raw, # Yaw
                                     m.chan5_raw,
                                     m.chan6_raw,
                                     m.chan7_raw,
                                     m.chan8_raw,
                                     m.chan9_raw,
                                     m.chan10_raw,
                                     m.chan11_raw,
                                     m.chan12_raw]
                elif (m.get_type() == 'GLOBAL_POSITION_INT'):
                    # print(m.lat/1e7, m.lon/1e7, m.alt/1e3)
                    self.uav_position = (
                        m.lat/1e7, m.lon/1e7, m.alt/1e3, int(m.hdg/1e2))
                    #self.print_debug(self.rc_channels)
        self.print_debug(">> MAVlink communication has stopped...")
        self.set_state("STOP")
        #min: 982
        #max: 2006
        #min: 0.5%
        #max: 12.5%
        #channel_11 = m.chan11
        #servo_pos = (channel_11 - 982)*(12/1024)
        #p.ChangeDutyCycle(servo_pos)

    def decode_armed(self, base_mode):
        return bool((base_mode & 0x80) >> 7)

    def decode_custom_flightmode(self, custom_mode):
        # get bits for sub and main mode
        sub_mode = (custom_mode >> 24)
        main_mode = (custom_mode >> 16) & 0xFF

        # set default value to none
        main_mode_text = ""
        sub_mode_text = ""

        # make list of modes
        mainmodeList = ["MANUAL", "ALTITUDE CONTROL", "POSITION CONTROL", "AUTO", "ACRO", "OFFBOARD", "STABILIZED", "RATTITUDE"]
        submodeList = ["READY", "TAKEOFF", "LOITER", "MISSION", "RTL", "LAND", "RTGS", "FOLLOW_TARGET", "PRECLAND"]

        # get mode from list based on index and what mode we have active. -1 as values is 0-indexed
        # print(main_mode)
        main_mode_text = mainmodeList[main_mode - 1]
        sub_mode_text = submodeList[sub_mode - 1]

        # if no submode, just return mainmode. As a result of -1 will wrap around the array and give precland
        if sub_mode <= 0 or sub_mode > 9:
            return main_mode_text
        else:
            return main_mode_text + " - " + sub_mode_text
    
    def run(self):
        '''
        This is the main loop function
        Add your code in here.
        '''

        # forever loop
        try:
            while True:
                if True:
                    self.capture()
                    if (abs(self.uav_position[2]-self.fouced_at_alt)>10):
                        self.fouced_at_alt = self.uav_position[2]
                        self.camera.set_controls({"AfTrigger": 0})
                        time.sleep(1)
                    if (self.uav_position[2]>10):
                        self.capture()
                    #if self.rc_channels[RC_CAPTURE] > 1500:
                        #self.capture()
                    #elif self.debounce:
                        #self.debounce = False
                    #self.print_debug("Testing: \n Armed: {} \n Mode {} \n RC Channels: {}".format(
                    #self.armed, self.mode, self.rc_channels))
                time.sleep(0.25)
                self.debounce = False
        # Kill MAVlink connection when Ctrl-C is pressed (results in a lock)
        except KeyboardInterrupt:
            self.print_debug('CTRL-C has been pressed! Exiting...')
            self.state = "EXIT"
            exit()


if __name__ == '__main__':
    DC = DroneControl()
    DC.print_debug('Brief pause before continuing....')
    time.sleep(1.0)
    DC.run()
        
