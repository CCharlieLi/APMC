from APMC.APMConnector import APMConnector


def main():
    vehicle = APMConnector('/dev/cu.usbserial-A503TM7S',57600)
    #vehicle.connect()
    vehicle.getAttributes()
    vehicle.addListeners()
    vehicle.armed()
    vehicle.getHomeLocation()
    vehicle.gerParams()
    vehicle.armAndTakeOff(100)


if __name__ == '__main__':
    main()