from dronekit import connect, VehicleMode, mavutil
from Callback import Callback
import time

class APMConnector:
    '''Class for connecting configuration '''

    def __init__(self, addr, baud):
        self.addr = addr
        self.baud = baud
        self.vehicle = None
        self.connect()
        self.selfCheck()

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
        pass
        
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
        self.Log("Global Location: %s" % self.vehicle.location.global_frame)
        self.Log("Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame)
        self.Log("Local Location: %s" % self.vehicle.location.local_frame)    #NED
        self.Log("Attitude: %s" % self.vehicle.attitude)
        self.Log("Velocity: %s" % self.vehicle.velocity)
        self.Log("GPS: %s" % self.vehicle.gps_0)
        self.Log("Groundspeed: %s" % self.vehicle.groundspeed)
        self.Log("Airspeed: %s" % self.vehicle.airspeed)
        self.Log("Gimbal status: %s" % self.vehicle.gimbal)
        self.Log("Battery: %s" % self.vehicle.battery)
        self.Log("EKF OK?: %s" % self.vehicle.ekf_ok)
        self.Log("Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        self.Log("Rangefinder: %s" % self.vehicle.rangefinder)
        self.Log("Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
        self.Log("Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)
        self.Log("Heading: %s" % self.vehicle.heading)
        self.Log("Is Armable?: %s" % self.vehicle.is_armable)
        self.Log("System status: %s" % self.vehicle.system_status.state)
        self.Log("Mode: %s" % self.vehicle.mode.name)    # settable
        self.Log("Armed: %s" % self.vehicle.armed)    # settable

    #
    # Add listener for attributes and parameters (Experiment)
    #
    def addListeners(self):
        self.isConnected()
        callback = Callback()
        self.Log('Adding attributes listeners ... ', 'SYSTEM')
        self.vehicle.add_attribute_listener('system_status.state', callback.cb_system_status)
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

        if self.vehicle.mode.name == "INITIALISING":
            self.Log("Waiting for vehicle to initialise ... ")
            time.sleep(1)
        while self.vehicle.gps_0.fix_type < 2:
            self.Log("Waiting for GPS...:" + self.vehicle.gps_0.fix_type)
            time.sleep(1)
        while not self.vehicle.is_armable:
            self.Log("Waiting for vehicle to initialise ... ")
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.mode.name=='GUIDED' or not self.vehicle.armed:
            self.Log('Getting ready to take off ...')
            time.sleep(1)
        self.Log("Armed: %s" % self.vehicle.armed)

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
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
                
        self.Log("Home location: %s" % self.vehicle.home_location)
        return self.vehicle.home_location

    #
    # Get vehicle parameters
    #
    def gerParams(self):
        self.isConnected()
        self.Log("Getting parameters ... ", 'SYSTEM')
        for key, value in self.vehicle.parameters.iteritems():
            self.Log(" Key:%s Value:%s" % (key,value))

    #
    # Take off to given altitude (Experiment)
    # Params:
    #     num: altitude (m)
    #
    def takeOff(self, altitude):
        self.isConnected()
        self.Log("Arming vehicle and taking off ... ", 'SYSTEM')

        self.armed()
        self.vehicle.simple_takeoff(altitude)
        self.Log('Taking off!', 'SYSTEM') 

        while True:
            print ">>> Altitude: ", self.vehicle.location.global_relative_frame.alt
            if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
                self.Log("Reached target altitude")
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
        self.Log("Going to ... ", 'SYSTEM')
        vehicle.mode = VehicleMode("GUIDED")

        a_location = LocationGlobalRelative(-34.364114, 149.166022, 30)
        self.vehicle..simple_goto(a_location, groundspeed = groundspeed)

    #
    # Set NED speed for UAV
    # Params:
    #     num: velocity_x (m/s)   velocity_x > 0 => fly North   velocity_x < 0 => fly South
    #     num: velocity_y (m/s)   velocity_y > 0 => fly East    velocity_y < 0 => fly West
    #     num: velocity_z (m/s)   velocity_z < 0 => ascend      velocity_z > 0 => descend
    #     num: duration (s)
    def sendNedVelocity(self, velocity_x, velocity_y, velocity_z, duration):
        self.isConnected()
        self.Log("Moving vehicle in direction based on specified velocity vectors ", 'SYSTEM')

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
            #self.Log("Airspeed: %s" % )
            self.Log("Currently, Global Location: %s, Airspeed: %s, " % (self.vehicle.location.global_frame,self.vehicle.airspeed))
            time.sleep(1)

    #
    # Set UAV yaw
    # Params:
    #     num:  degrees
    #     num:  direction (-1 ccw, 1 cw)
    #     bool: relative (relative offset 1, absolute angle 0)
    #
    #
    def setYaw(degrees, direction, relative=False):
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

    def get_location_metres(originalLocation, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
        else:
            raise Exception("Invalid Location object passed")

        return targetlocation;

    def get_distance_metres(aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def get_bearing(aLocation1, aLocation2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.

        This method is an approximation, and may not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing;

    def downloadMission(self):
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()

    def clearMission(self):
        cmds = vehicle.commands
        cmds.clear()
        cmds.upload()

    def setCommands(self):
        self.downloadMission()

        cmd1=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10)
        cmd2=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 10, 10, 10)
        cmds.add(cmd1)
        cmds.add(cmd2)
        cmds.upload() # Send commands

    def modifyCommands(self):
        # Get the set of commands from the vehicle
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()

        # Save the vehicle commands to a list
        missionlist=[]
        for cmd in cmds:
            missionlist.append(cmd)

        # Modify the mission as needed. For example, here we change the
        # first waypoint into a MAV_CMD_NAV_TAKEOFF command.
        missionlist[0].command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF

        # Clear the current mission (command is sent when we call upload())
        cmds.clear()

        #Write the modified mission and flush to the vehicle
        for cmd in missionlist:
            cmds.add(cmd)
        cmds.upload()

    def startMission():
        pass

    def gimbalRotate(self):
        self.isConnected()
        self.Log("Setting gimbal direction ... ", 'SYSTEM')
        self.vehicle.gimbal.rotate(-90, 0, 0)
        time.sleep(10)

    def gimbalTrack(self):
        self.isConnected()
        self.Log("Setting the camera to track point ... ", 'SYSTEM')
        self.vehicle.gimbal.target_location(self.vehicle.home_location)
        time.sleep(10)

    def set_roi(location):
        self.isConnected()
        self.Log("Setting ROI ... ", 'SYSTEM')
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

