#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rpi_capture_video_mavlink.py: Triggering video captures based on RC MAVLink messages 

Trigger works as a toggle on/off to start and stop videos, using the settings found in the definition section. 

"""

###############################################
# Definitions                                 #
###############################################
RC_CAPTURE = 6 # RX Channel 7


CAM_RESOLUTION  = (1920,1080)
CAM_FRAMERATE   = 30


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


###############################################
# RPi Imports                                 #
###############################################
import picamera
import datetime

###############################################
# Drone Control class                         #
###############################################
class DroneControl:
    def __init__(self, *args):
        self.debug = True
        self.ns = "DroneControl"
        
        self.armed = False
        self.mode = None
        self.rc_channels = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.index = 0
        self.debounce = False
        self.recording_video = False
        self.uav_position = None
        self.gps_loc_filename = 'gps_locations.log'

        self.state = "INIT"

        # Make New folder for photos
        self.folder_loc = '/home/pi/images/{}/'.format(datetime.datetime.now().strftime('%d-%b_%H.%M'))
        if not os.path.exists(self.folder_loc):
            os.mkdir(self.folder_loc)
        self.print_debug('Saving media to: {}'.format(self.folder_loc))

        self.camera = self.init_camera()  # change camera properties within this function

        # initialise MAVLink Connection
        # self.interface = mavutil.mavlink_connection("/dev/PX4", baud=115200, autoreconnect=True)

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
        camera = picamera.PiCamera()
        # camera.resolution = CAM_RESOLUTION  # RPI Cam 1
        camera.resolution = (3280, 2464) # RPI Cam 2
        #camera.shutter_speed = 10000 # in microseconds.  1/100 second
        camera.framerate = CAM_FRAMERATE
        return camera

    def print_debug(self,msg):
        ''' Print a message to the console with the prefix "[DroneControl]: '''
        if self.debug == True:
            print("[{}]: >> {} ".format(self.ns,msg))

    def set_state(self, data):
        self.state = data
        self.print_debug("New State: {}".format(data))

    def capture(self):
        if not self.debounce:
            filename = 'vid_'+str(self.index)+'.h264'
            if not self.recording_video:
                
                self.camera.start_recording(self.folder_loc+filename)

                self.print_debug('Recording Video to: {}'.format(filename))
                self.recording_video = True
                # save geolocation of image
                write_drone_location = open(
                self.folder_loc+self.gps_loc_filename, 'a')
                output = '{},{:.7f},{:.7f},{:.2f},{}'.format(
                    self.index,
                    self.uav_position[0],
                    self.uav_position[1],
                    self.uav_position[2],
                    self.uav_position[3],
                )

                write_drone_location.write(output)
                write_drone_location.close()
           
            else:   # if camera is still recording and switch pressed - toggle recording off
                self.camera.stop_recording()
                self.recording_video = False
                self.print_debug("Recording Stopped of Video: {}".format(filename))
                self.index += 1
            
            # debounce stops fast toggling of image/video capture
            self.debounce = True

    def recv_mavlink(self):
        while self.state == "RUNNING":
            m = self.interface.recv_msg()
            #print(m)
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
                # when RC switch is enabled.
                if self.rc_channels[RC_CAPTURE] > 1500:
                    self.capture()
                elif self.debounce:
                    self.debounce = False
                if self.debug:
                    if self.recording_video:
                        vid_out = "{} (vid_{}.h264)".format(self.recording_video, self.index)
                    else:
                        vid_out = self.recording_video
                    self.print_debug("\033c Testing: \n Armed: {} \n Mode {} \n Recording {} \n RC Channels: {} \n".format(self.armed, self.mode, vid_out, self.rc_channels))
                time.sleep(0.05)
        
        # Kill MAVlink connection when Ctrl-C is pressed (results in a lock)
        except KeyboardInterrupt:
            self.print_debug('CTRL-C has been pressed! Exiting...')
            if self.recording_video:
                self.print_debug('WARN >> Video still recording, stopping before exit.')
                self.camera.stop_recording()

            self.state = "EXIT"
            exit()

if __name__ == '__main__':
    DC = DroneControl()
    #can be removed if needed, just to add some "breathing space"
    DC.print_debug('Brief pause before continuing....')
    time.sleep(1.0)
    # main loop
    DC.run()