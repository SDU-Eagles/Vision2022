#!/usr/bin/env python3


from distutils.debug import DEBUG
from time import sleep
from pymavlink import mavwp
from pymavlink import mavutil
from pymavlink.mavutil import mavlink as mav_ref
import threading


'''
name: px4_mission_connect
maintainer: Oscar Bowen Schofield (obs@mmmi.sdu.dk)
date: 12th August 2022
'''
# used to debug elements of the script
DEBUG = False

# if enabled, the drone will fly to each waypoint and then take a "true north heading"
# if false, the drone will just be positioned in the heading to reach the current waypoint
use_recorded_yaw_heading = True

# waypoints filename and location, please make sure to change before flying.
gps_filename = '/home/pi/code/gps_locations.log'

''' 
Param MAV_CMD_NAV_WAYPOINT

1 - hold time (seconds)
2 - accept radius around waypoint (metres)
3 - pass radius (0 = hit waypoint, >0 pass by the waypoint be X metres) 
4 - Desired Yaw Heading in Degs. (NaN = system chooses yaw heading)
5 - Latitude 
6 - Longitude
7 - Altitude (m)
'''

class mav_handler():
    def __init__(self) -> None:
        super().__init__()

        self.ns = "DroneControl"
        self.debug = True

        self._status = 'init'
        self.connected = False
        self.uav_position = None
        self.seq_no = 0
        self.MAV_CMD_ACCEPT = None
        self.MAV_MISSION_ACCEPT = -1
        self.mission_request = -1
        self.cur_mission_wp = -1
        
        self.waypoint_list = []
        # connect to drone via pymavlink
        self.mav_comm = mavutil.mavlink_connection(
            "udp:127.0.0.1:14540", autoreconnect=True)
        # self.mav_comm = mavutil.mavlink_connection(
            # "/dev/PX4", baud=115200, autoreconnect=True)

        # waiting for heartbeat connection
        if self.mav_comm.wait_heartbeat(timeout=10):
            self.connected = True

        self.t_run = threading.Thread(target=self.receive_mavlink)
        self.t_run.start()

        self.print_debug('waiting for UAV connection')
        while(self.uav_position is None):
            sleep(0.1)

        self.set_state('ready')

    def print_debug(self, msg):
        ''' Print a message to the console with the prefix "[DroneControl]: '''
        if self.debug == True:
            print("[{}]: >> {} ".format(self.ns, msg))

    def decode_armed(self, base_mode):
        ''' Decodes the MAVLINK message for armed/disarmed status '''
        return bool((base_mode & 0x80) >> 7)

    def decode_custom_flightmode(self, custom_mode):
        '''
        Debugs the MAVLINK message for drone mode. 
        '''
        # get bits for sub and main mode
        sub_mode = (custom_mode >> 24)
        main_mode = (custom_mode >> 16) & 0xFF

        # set default value to none
        main_mode_text = ""
        sub_mode_text = ""

        # make list of modes
        mainmodeList = ["MANUAL", "ALTITUDE CONTROL", "POSITION CONTROL",
                        "AUTO", "ACRO", "OFFBOARD", "STABILIZED", "RATTITUDE"]
        submodeList = ["READY", "TAKEOFF", "LOITER", "MISSION",
                       "RTL", "LAND", "RTGS", "FOLLOW_TARGET", "PRECLAND"]

        # get mode from list based on index and what mode we have active. -1 as values is 0-indexed
        # print(main_mode)
        main_mode_text = mainmodeList[main_mode - 1]
        sub_mode_text = submodeList[sub_mode - 1]

        # if no submode, just return mainmode. As a result of -1 will wrap around the array and give precland
        if sub_mode <= 0 or sub_mode > 9:
            return main_mode_text
        else:
            return main_mode_text + " - " + sub_mode_text

    def set_state(self, new_state) -> bool:
        '''
        Internal state counter, used for checking when to accept MAVLINK Messages.
        '''
        self._status = new_state.upper()
        if self.debug:
            self.print_debug('new_state: %s' % self._status)
        return self._status

    def receive_mavlink(self):
        while(self.connected):
            m = self.mav_comm.recv_msg()
            if m:
                if (m.get_type() == 'HEARTBEAT'):
                    self.armed = self.decode_armed(m.base_mode)
                    self.mode = self.decode_custom_flightmode(m.custom_mode)

                elif (m.get_type() == 'GLOBAL_POSITION_INT'):
                    # print(m.lat/1e7, m.lon/1e7, m.alt/1e3)
                    self.uav_position = (
                        m.lat/1e7, m.lon/1e7, m.alt/1e3, int(m.hdg/1e2))

                elif (m.get_type() == 'MAV_CMD_IMAGE_START_CAPTURE'):
                    self.print_debug('image capture detected')
                elif (m.get_type() == 'COMMAND_ACK'):
                    self.MAV_CMD_ACCEPT = m.result
                elif (m.get_type() == 'MISSION_REQUEST'):
                    self.mission_request = m.seq
                    if DEBUG:
                        print(m)
                elif (m.get_type() == 'MISSION_ACK'):
                    self.MAV_MISSION_ACCEPT = m.type
                elif (m.get_type() == 'MISSION_ITEM_REACHED'):
                    if self.cur_mission_wp != m.seq:
                        msg = "ping camera at wp{}".format(m.seq)
                        # print(msg)
                        self.cur_mission_wp = m.seq
        print(">> MAVlink communication has stopped...")
        self.set_state("STOP")

    def wait_for_ack(self):
        while self.MAV_CMD_ACCEPT is None:
            sleep(0.01)

        if self.MAV_CMD_ACCEPT is mav_ref.MAV_RESULT_ACCEPTED:
            output = True
        elif self.MAV_CMD_ACCEPT is mav_ref.MAV_RESULT_TEMPORARILY_REJECTED:
            self.print_debug(
                'Command would be accepted, but system is not ready to accept')
            output = False
        else:
            output = False
        self.MAV_CMD_ACCEPT = None
        return output

    def wait_for_mission_ack(self):
        while self.MAV_MISSION_ACCEPT is None:
            sleep(0.01)

        if self.MAV_MISSION_ACCEPT is mav_ref.MAV_MISSION_ACCEPTED:
            output = True
        elif self.MAV_MISSION_ACCEPT is mav_ref.MAV_MISSION_ERROR:
            self.print_debug(
                'Command would be accepted, but system is not ready to accept')
            output = False
        else:
            output = False
        return output

    def set_takeoff(self, alt=10):
        '''
        set_takeoff: Generate a mission item to command a takeoff
        input: altitude (default 10 metres)
        output: mavlink mission item message   
        '''
        if self.seq_no == 0:
            current = 0
        else:
            current = self.seq_no - 1

        msg = mavutil.mavlink.MAVLink_mission_item_message(
            self.mav_comm.target_system,
            self.mav_comm.target_component,
            self.seq_no,   # sequence number
            mav_ref.MAV_FRAME_GLOBAL_RELATIVE_ALT,            # Frame
            mav_ref.MAV_CMD_NAV_TAKEOFF,
            1,  # current
            1,  # autocontinue
            15.0,                      # min pitch
            0,                      # empty
            0,                      # empty
            float('nan'),           # yaw
            self.uav_position[0],   # lat
            self.uav_position[1],   # lon
            float(alt))  # alt
        if DEBUG:
            print('launch \t seq{} cmd:{} \t Lat:{:.7f} \t Lon: {:.7f} \t Alt:{:.3f}'.format(
            self.seq_no, mav_ref.MAV_CMD_NAV_TAKEOFF,
            self.uav_position[0], self.uav_position[1], alt))

        return msg

    def set_waypoint(self, waypoint_entry, hold_time=5.0, wp_yaw=float('nan')):
        '''
        set_waypoint
        This function converts an input string into a waypoint mission item
        inputs:
        - waypoint_entry -> comma-separated waypoint (Latitude,Longitude,Altitude)
        '55.472507,10.324265,5.0'
        - hold_time: how long it should wait at each waypoint in seconds (default 5 seconds)
        - wp_yaw = the heading at which the drone should take at each  

        '''

        wp_info = waypoint_entry.rsplit(',')
        command = mav_ref.MAV_CMD_NAV_WAYPOINT
        param1 = hold_time  # hold time (seconds)
        param2 = 0  # accept radius
        param3 = 0  # pass by radius
        param4 = float(wp_yaw)  # Yaw
        wp_lat = float(wp_info[1])
        wp_lon = float(wp_info[2])
        wp_alt = float(wp_info[3])

        msg = mavutil.mavlink.MAVLink_mission_item_message(
            self.mav_comm.target_system,
            self.mav_comm.target_component,
            self.seq_no,   # sequence number
            mav_ref.MAV_FRAME_GLOBAL_RELATIVE_ALT,            # Frame
            command,
            0, 1,       # current and autocontinue
            param1, param2, param3, param4,
            wp_lat, wp_lon, wp_alt)
        if DEBUG:
            print('wp \t seq{} cmd:{} \t Lat:{:.7f} \t Lon: {:.7f} \t Alt:{:.3f} \t Yaw:{}'.format(
                self.seq_no, command, wp_lat, wp_lon, wp_alt, param4))
        return msg

    def gen_rtl_landing(self):
        '''
        gen_rtl_landing: This function generates the mission entry 
        for the drone to Return To Launch (RTL).
        input: None
        output: mission item with  
        '''
        msg = mavutil.mavlink.MAVLink_mission_item_message(
            self.mav_comm.target_system,
            self.mav_comm.target_component,
            self.seq_no,   # sequence number
            mav_ref.MAV_FRAME_MISSION,            # Frame
            20,
            0, 0,           # hold time/radius kept empty
            0, 0, 0, 0, 0, 0, 0)  # params 1-7 left empty
        if DEBUG:
            print('LAND \t seq{} cmd:{}'.format(
                self.seq_no, 20,))
        return msg

    def generate_mission(self):
        '''
        Generates a mission list based on a set of waypoints outlined at the top of the script. 
        Also includes takeoff and landing commands, with the whole mission being transmitted to the drone.
        Returns Acceptance or rejection message to the console 
        '''
        self.seq_no = 0

        self.waypoint_list=[]
        with open(gps_filename) as f:
            for i, line in enumerate(f):
                self.waypoint_list.append(line)
                if DEBUG:
                    print(self.waypoint_list[i])
            
        mission_wp = mavwp.MAVWPLoader()
        self.print_debug('\nGenerating mission...')

        takeoff_wp = self.set_takeoff()
        # create takeoff command
        mission_wp.add(takeoff_wp)
        self.seq_no += 1

        for waypoint in self.waypoint_list:
            if use_recorded_yaw_heading:
                # use yaw heading to get to the waypoint
                pkg_waypoint_noyaw = self.set_waypoint(waypoint, hold_time=0)
                mission_wp.add(pkg_waypoint_noyaw)
                self.seq_no += 1
                #stay at position and take photo in the desired yaw heading
                waypoint_split=waypoint.rsplit(',')
                pkg_waypoint = self.set_waypoint(
                    waypoint, wp_yaw=waypoint_split[4])
            
            else:
                pkg_waypoint = self.set_waypoint(waypoint)
            mission_wp.add(pkg_waypoint)    
            self.seq_no += 1

        mission_wp.add(self.gen_rtl_landing())
        print("")

        self.mav_comm.waypoint_clear_all_send()
        self.mav_comm.waypoint_count_send(mission_wp.count())
        
        while self.mission_request != mission_wp.count():
            # listens to flight controller requests for mission items
            while self.mission_request == -1:
                sleep(0.01)
            self.print_debug('Sending mission item {} of {}'.format(
                self.mission_request+1, mission_wp.count()))
            self.mav_comm.mav.send(mission_wp.wp(self.mission_request))
            if self.wait_for_mission_ack() and self.mission_request == mission_wp.count()-1:
                break
            else:
                sleep(0.05)

        if self.MAV_MISSION_ACCEPT == mav_ref.MAV_MISSION_ACCEPTED:
            self.print_debug('Mission sent successfully')

        elif self.MAV_CMD_ACCEPT == mav_ref.MAV_MISSION_REJECTED:
            self.print_debug('Mission Was Rejected.')

    def start_mission(self):
        '''
        This function sets the drone into mission mode, and commands for mission start.
        Only use this if you know what youre doing!
        '''
        # Set drone mode to 'Mission'
        self.mav_comm.mav.command_long_send(
            self.mav_comm.target_system,
            self.mav_comm.target_component,
            mav_ref.MAV_CMD_DO_SET_MODE,
            0,
            4,  # auto
            4,  # MISSION
            0, 0, 0, 0, 0
        )

        self.mav_comm.mav.command_long_send(
            self.mav_comm.target_system,
            self.mav_comm.target_component,
            mav_ref.MAV_CMD_MISSION_START,
            0, 0,
            0, 0, 0, 0, 0, 0, 0
        )

    def run(self):
        try:
            sleep(1.0)
            self.generate_mission()

            while True:
                sleep(1.0)
        except KeyboardInterrupt:
            print('CTRL-C has been pressed! Exiting...')
            self.connected = False
            self.state = "EXIT"
            exit()

mh = mav_handler()
mh.run()
