#!/usr/bin/env python3
# -*- coding: utf-8 -*-

###############################################
# Standard Imports                            #
###############################################
import time
import threading
import os

###############################################
# RPi Imports                                 #
###############################################
from picamera2 import Picamera2
import datetime

###############################################
# Drone Control class                         #
###############################################

import numpy as np
import os

class DroneControl:
    def __init__(self, *args):
        self.ns = "DroneControl"
        self.armed = False
        self.mode = None
        self.rc_channels = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.index = 0
        self.debounce = False
        self.uav_position = [0,0,0,0]
        self.debug = True

        self.state = "INIT"

        # Make New folder for photos
        self.folder_loc = '/home/pi/Pictures/{}/'.format(datetime.datetime.now().strftime('%d-%b_%H.%M'))
        if not os.path.exists(self.folder_loc):
            os.mkdir(self.folder_loc)
        self.print_debug('Saving media to: {}'.format(self.folder_loc))
        
        self.camera = self.init_camera()  # change camera properties within this function#       # initialise MAVLink Connection
        
    def init_camera(self):
        '''
        Initialise the raspberry pi camera with desired properties. 
        '''
        camera = Picamera2()
        config = camera.create_still_configuration()
        camera.configure(config)
        camera.resolution = (4056, 3040)    # RPI HQ Cam
        camera.framerate = 60
        camera.start()
        
        return camera

    def print_debug(self,msg):
        ''' Print a message to the console with the prefix "[DroneControl]: '''
        if self.debug == True:
            print("[{}]: >> {} ".format(self.ns,msg))

    def set_state(self, data):
        self.state = data
        self.print_debug("New State: {}".format(data))

    def capture(self):
        'captures images and gps locations of said images'
        if not self.debounce:
            img_loc = self.folder_loc+'img_'+str(self.index)
            self.camera.capture_file(img_loc+'.jpg')
            msg="Image captured. Saved image as img_{}.jpg".format(self.index)
            self.print_debug(msg)
            
            self.index += 1
            self.debounce = True

    
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
                time.sleep(0.25)
                self.debounce = False

        # Kill MAVlink connection when Ctrl-C is pressed (results in a lock)
        except KeyboardInterrupt:
            self.print_debug('CTRL-C has been pressed! Exiting...')
            self.state = "EXIT"
            self.camera.close()
            exit()


if __name__ == '__main__':
    DC = DroneControl()
    DC.print_debug('Brief pause before continuing....')
    time.sleep(1.0)
    DC.run()

