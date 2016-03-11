from dronekit import connect, VehicleMode, mavutil, Command
from Callback import Callback
import time
import sys
reload(sys)
sys.setdefaultencoding( 'utf-8' )

class APMConnector:
    '''Class for connecting configuration '''

    def __init__(self, addr, baud):
        self.addr = addr
        self.baud = baud
        self.vehicle = None
        self.connect()
        self.selfCheck()

    #########################################
    #                                       #
    #            Initial functions          #
    #                                       #
    #########################################

    # 
    # Check if connection is established
    # 
    def isConnected(self):
        if self.vehicle == None:
            self.Log('Please connect to a vehicle first!', 'WARN')
            exit(0)
            #return False
        return True

    # 
    # Connect board with given serial/network port and baudrate
    # Return: 
    #     object: Vehicle object
    # 
    def connect(self):
        self.Log('Connecting to the vehicle ... ', 'SYSTEM')
        self.vehicle = connect(self.addr, baud=self.baud, wait_ready=True)
        return self.vehicle

    #
    # UAV self check (Experiment)
    #
    def selfCheck(self):
        #[TODO]
        pass
        


    #########################################
    #                                       #
    #            Control functions          #
    #                                       #
    #########################################



    # 
    # Set UAV speed (Experiment)
    # Params:
    #     num: Air speed (m/s)
    #     num: Ground speed (m/s)
    # Return:
    #     (num, num): (airspeed, groundspeed) 
    # 
    def setSpeed(self, airspeed, groundspeed):
        self.isConnected()
        self.Log('Setting attributes ... ', 'SYSTEM')

        vehicle.airspeed = airspeed #m/s
        vehicle.groundspeed = groundspeed #m/s

        self.Log('Air speed: %s, Ground speed: %s' % (vehicle.airspeed, vehicle.groundspeed))
        return (vehicle.airspeed, vehicle.groundspeed)

    #
    # Get UAV attributes (Experiment)
    #
    def getAttributes(self):
        self.isConnected()
        self.Log('Reading attribute ... ', 'SYSTEM')
        self.Log('Global Location: %s' % self.vehicle.location.global_frame)
        self.Log('Global Location (relative altitude): %s' % self.vehicle.location.global_relative_frame)
        self.Log('Local Location: %s' % self.vehicle.location.local_frame)    #NED
        self.Log('Attitude: %s' % self.vehicle.attitude)
        self.Log('Velocity: %s' % self.vehicle.velocity)
        self.Log('GPS: %s' % self.vehicle.gps_0)
        self.Log('Groundspeed: %s' % self.vehicle.groundspeed)
        self.Log('Airspeed: %s' % self.vehicle.airspeed)
        self.Log('Gimbal status: %s' % self.vehicle.gimbal)
        self.Log('Battery: %s' % self.vehicle.battery)
        self.Log('EKF OK?: %s' % self.vehicle.ekf_ok)
        self.Log('Last Heartbeat: %s' % self.vehicle.last_heartbeat)
        self.Log('Rangefinder: %s' % self.vehicle.rangefinder)
        self.Log('Rangefinder distance: %s' % self.vehicle.rangefinder.distance)
        self.Log('Rangefinder voltage: %s' % self.vehicle.rangefinder.voltage)
        self.Log('Heading: %s' % self.vehicle.heading)
        self.Log('Is Armable?: %s' % self.vehicle.is_armable)
        self.Log('System status: %s' % self.vehicle.system_status.state)
        self.Log('Mode: %s' % self.vehicle.mode.name)    # settable
        self.Log('Armed: %s' % self.vehicle.armed)    # settable

    #
    # Add listener for attributes and parameters (Experiment)
    #
    def addListeners(self):
        self.isConnected()
        callback = Callback()
        self.Log('Adding attributes listeners ... ', 'SYSTEM')
        self.vehicle.add_attribute_listener('location.global_frame', callback.cb_global_frame)
        self.vehicle.add_attribute_listener('location.global_relative_frame', callback.cb_global_relative_frame)
        self.vehicle.add_attribute_listener('local_frame', callback.cb_local_frame)
        self.vehicle.add_attribute_listener('attitude', callback.cb_attitude)
        self.vehicle.add_attribute_listener('velocity', callback.cb_velocity)
        self.vehicle.add_attribute_listener('gps_0', callback.cb_gps_0)
        self.vehicle.add_attribute_listener('gimbal', callback.cb_gimbal)
        self.vehicle.add_attribute_listener('battery', callback.cb_battery)
        self.vehicle.add_attribute_listener('rangefinder', callback.cb_rangerfinder)
        self.vehicle.add_attribute_listener('ekf_ok', callback.cb_ekf_ok)
        self.vehicle.add_attribute_listener('last_heartbeat', callback.cb_last_heartbeat)
        self.vehicle.add_attribute_listener('home_location', callback.cb_home_location)
        self.vehicle.add_attribute_listener('system_status.state', callback.cb_system_status)
        self.vehicle.add_attribute_listener('heading', callback.cb_heading)
        self.vehicle.add_attribute_listener('is_armable', callback.cb_is_armable)
        self.vehicle.add_attribute_listener('airspeed', callback.cb_airspeed)
        self.vehicle.add_attribute_listener('groundspeed', callback.cb_groundspeed)
        self.vehicle.add_attribute_listener('armed', callback.cb_armed)
        self.vehicle.add_attribute_listener('mode.name', callback.cb_mode)
        self.Log('Adding parameters listeners ... ', 'SYSTEM')
        self.vehicle.parameters.add_attribute_listener('*', callback.cb_param_all)

    #
    # Arm UAV
    #
    def armed(self):
        self.isConnected()
        self.Log('Arming ... ', 'SYSTEM')

        if self.vehicle.mode.name == 'INITIALISING':
            self.Log('Waiting for vehicle to initialise ... ')
            time.sleep(1)
        while self.vehicle.gps_0.fix_type < 2:
            self.Log('Waiting for GPS...:' + self.vehicle.gps_0.fix_type)
            time.sleep(1)
        while not self.vehicle.is_armable:
            self.Log('Waiting for vehicle to initialise ... ')
            time.sleep(1)

        self.vehicle.mode = VehicleMode('GUIDED')
        self.vehicle.armed = True

        while not self.vehicle.mode.name=='GUIDED' or not self.vehicle.armed:
            self.Log('Getting ready to take off ...')
            time.sleep(1)
        self.Log('Armed: %s' % self.vehicle.armed)

    # 
    # Get home location (Experiment)
    # Return:
    #     location
    # 
    def getHomeLocation(self):
        self.isConnected()
        self.Log('Getting Vehicle Home location ... ', 'SYSTEM')

        while not self.vehicle.home_location:
            self.Log('Waiting for home location ... ')
            self.downloadMission()
                
        self.Log('Home location: %s' % self.vehicle.home_location)
        return self.vehicle.home_location

    #
    # Get vehicle parameters
    #
    def gerParams(self):
        self.isConnected()
        self.Log('Getting parameters ... ', 'SYSTEM')
        for key, value in self.vehicle.parameters.iteritems():
            self.Log(' Key:%s Value:%s' % (key,value))

    #
    # Take off to given altitude (Experiment)
    # Params:
    #     num: altitude (m)
    #
    def takeOff(self, altitude):
        self.isConnected()
        self.Log('Arming vehicle and taking off ... ', 'SYSTEM')

        self.armed()
        self.vehicle.simple_takeoff(altitude)
        self.Log('Taking off!', 'SYSTEM') 

        while True:
            print '>>> Altitude: ', self.vehicle.location.global_relative_frame.alt
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                self.Log('Reached target altitude')
                break
            time.sleep(1)

    #
    # Go to given location with given groud speed (Experiment)
    # Params:
    #     location: location
    #     num: groundspeed (m/s)
    #
    def goTo(self, location, groundspeed):
        self.isConnected()
        self.Log('Going to ... ', 'SYSTEM')
        vehicle.mode = VehicleMode('GUIDED')

        a_location = LocationGlobalRelative(-34.364114, 149.166022, 30)
        self.vehicle.simple_goto(a_location, groundspeed = groundspeed)

    #
    # Set NED speed for UAV
    # Params:
    #     num: velocity_x (m/s)   velocity_x > 0 => fly North   velocity_x < 0 => fly South
    #     num: velocity_y (m/s)   velocity_y > 0 => fly East    velocity_y < 0 => fly West
    #     num: velocity_z (m/s)   velocity_z < 0 => ascend      velocity_z > 0 => descend
    #     num: duration (s)
    def sendNedVelocity(self, velocity_x, velocity_y, velocity_z, duration):
        self.isConnected()
        self.Log('Moving vehicle in direction based on specified velocity vectors ', 'SYSTEM')

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            #self.Log('Airspeed: %s' % )
            self.Log('Currently, Global Location: %s, Airspeed: %s, ' % (self.vehicle.location.global_frame,self.vehicle.airspeed))
            time.sleep(1)

    #
    # Set UAV yaw
    # Params:
    #     num:  degrees (0 degrees is North in absolute angle)
    #     num:  direction (-1 ccw, 1 cw)
    #     bool: relative (relative offset 1, absolute angle 0)
    #
    # Note:(http://python.dronekit.io/guide/copter/guided_mode.html)
    # - The yaw will return to the default (facing direction of travel) after you set the mode or change the command 
    #   used for controlling movement.
    # - At time of writing there is no safe way to return to the default yaw 'face direction of travel' behaviour.
    # - After taking off, yaw commands are ignored until the first 'movement' command has been received. If you need 
    #   to yaw immediately following takeoff then send a command to 'move' to your current position.
    # - Setting the ROI may work to get yaw to track a particular point (depending on the gimbal setup).

    def setYaw(degrees, direction, relative=False):
        self.isConnected()
        self.Log('Setting Yaw ... ', 'SYSTEM')

        if relative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        msg = vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            degrees,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            direction,  # param 3, direction 
            is_relative,# param 4, 
            0, 0, 0)    # param 5 ~ 7 not used

        self.vehicle.send_mavlink(msg)

    #########################################
    #                                       #
    #            Mission functions          #
    #                                       #
    #########################################

    #
    # Download commands(mission) from Pixhawk
    # Return:
    #     CommandSequence
    #
    def downloadMission(self):
        self.isConnected()
        self.Log('Downloading mission ... ', 'SYSTEM')

        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        return cmds


    #
    # Clear mission in Pixhawk
    # 
    # Note:
    # If a mission that is underway is cleared, the mission will continue to the next waypoint. 
    # If you don't add a new command before the waypoint is reached then the vehicle mode will
    # change to RTL (return to launch) mode.
    #
    def clearMission(self):
        self.isConnected()
        self.Log('Clearing mission ... ', 'SYSTEM')

        # [TODO] check commands exist
        cmds = vehicle.commands
        cmds.clear()
        cmds.upload()

    #
    # Set mission
    #
    def setMission(self, commands):
        self.isConnected()
        self.Log('Setting mission ... ', 'SYSTEM')

        cmds = self.downloadMission()
        cmd1=Command( 0, 0, 0, 
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 0, 0, 0, 0, 0, 0, 0, 10)
        cmd2=Command( 0, 0, 0, 
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 
            0, 0, 0, 0, 0, 0, 10, 10, 10)
        cmds.add(cmd1)
        cmds.add(cmd2)
        cmds.upload() # Send commands

    #
    # Modify mission
    #
    def modifyMission(self):
        self.isConnected()
        self.Log('Modifying mission ... ', 'SYSTEM')

        cmds = self.downloadMission()
        missionlist = []
        for cmd in cmds:
            missionlist.append(cmd)

        # Modify the mission as needed. For example, here we change the
        # first waypoint into a MAV_CMD_NAV_TAKEOFF command.
        missionlist[0].command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        cmds.clear()

        for cmd in missionlist:
            cmds.add(cmd)
        cmds.upload()

    #
    # Start mission
    #
    def startMission(self):
        self.isConnected()
        self.Log('Starting mission ... ', 'SYSTEM')

        self.vehicle.mode = VehicleMode('AUTO')
        # [TODO] Need listener for self.vehicle.commands.next 
        # [TODO] Check if reach end of the mission (check model or waypoint?)

    #
    # Pause mission
    #
    def pauseMission(self):
        self.isConnected()
        self.Log('Pausing mission ... ', 'SYSTEM')

        self.vehicle.mode = VehicleMode('GUIDED')
        # [TODO] Set MIS_RESTART as 0

    #
    # Stop mission
    #
    def stopMission(self):
        self.isConnected()
        self.Log('Stopping mission ... ', 'SYSTEM')

        self.vehicle.mode = VehicleMode('GUIDED')
        # [TODO] Set MIS_RESTART as 1

    #
    # Upload a mission from a file
    # Imput:
    #     file: mission file
    #
    def importMission(self, fileName):
        self.isConnected()
        self.Log('Importing mission ... ', 'SYSTEM')

        missionlist = self.readMission(fileName)
        self.clearMission()
        for command in missionlist:
            cmds.add(command)
        print ' Upload mission'
        vehicle.commands.upload()

    #
    # Load a mission from a file into a list
    #
    def readMission(self, fileName):
        self.isConnected()
        self.Log('Reading mission file ... ', 'SYSTEM')

        cmds = self.vehicle.commands
        missionlist=[]
        with open(fileName) as f:
            for i, line in enumerate(f):
                if i==0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:
                    linearray=line.split('\t')
                    ln_index=int(linearray[0])
                    ln_currentwp=int(linearray[1])
                    ln_frame=int(linearray[2])
                    ln_command=int(linearray[3])
                    ln_param1=float(linearray[4])
                    ln_param2=float(linearray[5])
                    ln_param3=float(linearray[6])
                    ln_param4=float(linearray[7])
                    ln_param5=float(linearray[8])
                    ln_param6=float(linearray[9])
                    ln_param7=float(linearray[10])
                    ln_autocontinue=int(linearray[11].strip())
                    cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                    missionlist.append(cmd)
        return missionlist

    #
    # Save a mission in the Waypoint file format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    #
    def save_mission(self, fileName):
        self.isConnected()
        self.Log('Saving mission into file %s ... ' % fileName, 'SYSTEM')

        cmds = self.downloadMission()
        missionlist=[]
        for cmd in cmds:
            missionlist.append(cmd)

        output='QGC WPL 110\n'
        for cmd in missionlist:
            commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
            output+=commandline
        with open(aFileName, 'w') as file_:
            file_.write(output)


    #########################################
    #                                       #
    #            Gimbal  functions          #
    #                                       #
    #########################################


    def gimbalRotate(self):
        self.isConnected()
        self.Log('Setting gimbal direction ... ', 'SYSTEM')

        self.vehicle.gimbal.rotate(-90, 0, 0)
        time.sleep(10)

    def gimbalTrack(self):
        self.isConnected()
        self.Log('Setting the camera to track point ... ', 'SYSTEM')

        self.vehicle.gimbal.target_location(self.vehicle.home_location)
        time.sleep(10)

    #
    # Point camera gimbal at a specified region of interest (LocationGlobal)
    # 
    # Note:
    # - The ROI (and yaw) is also reset when the mode, or the command used to control movement, is changed.
    #
    def setROI(self, location):
        self.isConnected()
        self.Log('Setting ROI ... ', 'SYSTEM')

        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
            0, #confirmation
            0, 0, 0, 0, #params 1-4
            location.lat,
            location.lon,
            location.alt
            )
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


    #########################################
    #                                       #
    #       Frame conversion functions      #
    #                                       #
    #########################################
    
    # (Experiment)
    # Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    # specified `originalLocation`. The returned LocationGlobal has the same `alt` value
    # as `originalLocation`.
    #
    # The function is useful when you want to move the vehicle around specifying locations relative to
    # the current vehicle position.
    #
    # The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    #
    # For more information see:
    # http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    def getLocationMeters(self, originalLocation, dNorth, dEast):
        earth_radius=6378137.0 #Radius of 'spherical' earth

        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi * originalLocation.lat/180))

        #New position in decimal degrees
        newlat = originalLocation.lat + (dLat * 180/math.pi)
        newlon = originalLocation.lon + (dLon * 180/math.pi)
        if type(originalLocation) is LocationGlobal:
            targetLocation = LocationGlobal(newlat, newlon, originalLocation.alt)
        elif type(originalLocation) is LocationGlobalRelative:
            targetLocation = LocationGlobalRelative(newlat, newlon, originalLocation.alt)
        else:
            raise Exception('Invalid Location object passed')

        return targetLocation;

    # (Experiment)
    # Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.
    #
    # This method is an approximation, and will not be accurate over large distances and close to the
    # earth's poles. It comes from the ArduPilot test code:
    # https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    #
    def getDistanceMeters(self, aLocation1, aLocation2):
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    #
    # Gets distance in metres to the current waypoint.
    # It returns None for the first waypoint (Home location).
    #
    def distanceToCurrentWaypoint(self):
        nextWayPoint = self.vehicle.commands.next
        if nextWayPoint == 0:
            return None
        missionItem = self.vehicle.commands[nextWayPoint-1] #commands are zero indexed
        lat = missionItem.x
        lon = missionItem.y
        alt = missionItem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
        distanceToPoint = getDistanceMeters(self.vehicle.location.global_frame, targetWaypointLocation)
        return distanceToPoint

    # (Experiment)
    # Returns the bearing between the two LocationGlobal objects passed as parameters.
    # 
    # This method is an approximation, and may not be accurate over large distances and close to the
    # earth's poles. It comes from the ArduPilot test code:
    # https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    # 
    def getBearing(self, aLocation1, aLocation2):
        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing;


    #
    # Log function
    #
    def Log(self, content, level=None):
        if level == 'SYSTEM':
            print '\033[0;33;40m'
            print ''' %s   -   %s''' % (level, content)
            print '\033[0m'
        elif level == 'WARN':
            print '\033[1;31;40m'
            print ''' %s   -   %s''' % (level, content)
            print '\033[0m'
        else:
            print content

        # write in log file

