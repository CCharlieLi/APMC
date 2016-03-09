
class Callback:
    def __init__(self):
        pass
    """Functions for attributes callback"""
    def cb_global_frame(self, vehicle, attr, value):
        print "Attribute Update - Global Location: %s" % value

    def cb_system_status(self, vehicle, attr, value):
        print "Attribute Update - System status: %s" % value

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