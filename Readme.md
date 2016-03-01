### What's this
This is a [Dronekit](https://github.com/dronekit/dronekit-python) based APM connector lib, we use it on our companion computer(PBI-M2/ODROID XU4) to control Pixhawk/APM.

### Test

- Connect the pixhawk with your companion computer.
- Set the right port and baudrate for connection in `test.py`. Like:

```
APMConnector('/dev/cu.usbserial-A503TM7S',57600)
``` 

- Test
```
$ python test.py
```

### License
MIT

### Authors
CCharlieLi
