
class Callback:
    def __init__(self):
        pass
    """Functions for attributes callback"""
    def cb_version(self, vehicle, attr, value):
        print "Attribute Update - Version: %s" % value

    def cb_capabilities(self, vehicle, attr, value):
        print "Attribute Update - Capabilities: %s" % value

    def cb_global_frame(self, vehicle, attr, value):
        print "Attribute Update - %s" % value

    def cb_global_relative_frame(self, vehicle, attr, value):
        print "Attribute Update - %s" % value

    def cb_local_frame(self, vehicle, attr, value):
        print "Attribute Update - Local Location: %s" % value

    def cb_attitude(self, vehicle, attr, value):
        print "Attribute Update - %s" % value

    def cb_velocity(self, vehicle, attr, value):
        print "Attribute Update - Velocity: %s" % value

    def cb_gps_0(self, vehicle, attr, value):
        print "Attribute Update - %s" % value

    def cb_gimbal(self, vehicle, attr, value):
        print "Attribute Update - Gimbal: %s" % value

    def cb_battery(self, vehicle, attr, value):
        print "Attribute Update - %s" % value

    def cb_rangerfinder(self, vehicle, attr, value):
        print "Attribute Update - RangerFinder: %s" % value

    def cb_ekf_ok(self, vehicle, attr, value):
        print "Attribute Update - EKF_OK: %s" % value

    def cb_last_heartbeat(self, vehicle, attr, value):
        print "Attribute Update - Last Heartbeat: %s" % value

    def cb_home_location(self, vehicle, attr, value):
        print "Attribute Update - Home Location: %s" % value

    def cb_system_status(self, vehicle, attr, value):
        print "Attribute Update - System status: %s" % value

    def cb_heading(self, vehicle, attr, value):
        print "Attribute Update - Heading: %s" % value

    def cb_is_armable(self, vehicle, attr, value):
        print "Attribute Update - Is Armable: %s" % value

    def cb_airspeed(self, vehicle, attr, value):
        print "Attribute Update - Airspeed: %s" % value

    def cb_groundspeed(self, vehicle, attr, value):
        print "Attribute Update - Groundspeed: %s" % value

    def cb_armed(self, vehicle, attr, value):
        print "Attribute Update - Armed: %s" % value

    def cb_mode(self, vehicle, attr, value):
        print "Attribute Update - Mode: %s" % value

    ######## [TODO: Add more attribute listeners callback] ########

    def cb_attr_all(self, vehicle, attr, value):
        print "ATTRIBUTE CALLBACK: %s changed to: %s" % (attr,value)
    
    """Functions for parameters callback"""

    ######## [TODO: Add more parameters listeners callback] ########

    def cb_param_all(self, param_name, value):
        print "PARAMETER CALLBACK: %s changed to: %s" % (param, value)