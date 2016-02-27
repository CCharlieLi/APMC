from APMC.APMConnector import APMConnector


def main():
    vehicle = APMConnector('127.0.0.1:14550')
    #vehicle.connect()
    vehicle.getAttributes()
    vehicle.addListeners()
    vehicle.armed()
    vehicle.getHomeLocation()
    vehicle.gerParams()
    vehicle.armAndTakeOff(100)

def test():
    print "Start simulator (SITL)"
    from dronekit_sitl import SITL
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,353']
    sitl.launch(sitl_args, await_ready=True, restart=True)

    # Import DroneKit-Python
    from dronekit import connect, VehicleMode
    import time

    # Connect to the Vehicle.
    print "Connecting to vehicle on: 'tcp:127.0.0.1:5760'"
    vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

    # Get some vehicle attributes (state)
    print "Get some vehicle attribute values:"
    print " GPS: %s" % vehicle.gps_0
    print " Battery: %s" % vehicle.battery
    print " Last Heartbeat: %s" % vehicle.last_heartbeat
    print " Is Armable?: %s" % vehicle.is_armable
    print " System status: %s" % vehicle.system_status.state
    print " Mode: %s" % vehicle.mode.name    # settable

    # Close vehicle object before exiting script
    vehicle.close()

    # Shut down simulator
    sitl.stop()
    print("Completed")

if __name__ == '__main__':
    main()