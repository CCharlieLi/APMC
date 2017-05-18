from APMC.APMConnector import APMConnector


def main():
    vehicle = APMConnector('/dev/cu.usbmodem1',57600)
    #vehicle.connect()
    # vehicle.getAttributes()
    #vehicle.addListeners()
    # vehicle.armed()
    # vehicle.getHomeLocation()
    vehicle.gerParams()
    #vehicle.setMission(1)
    #vehicle.startMission()
    #vehicle.takeOff(0.5)
    #vehicle.setYaw(90,-1,1)
    #vehicle.sendNedVelocity(-1,0,0,5)


if __name__ == '__main__':
    main()