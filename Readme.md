# What's this
This is a [Dronekit](http://python.dronekit.io/about/overview.html) based Pixhawk/APM connector lib in python, we use it on our companion computer(PBI-M2/ODROID XU4) to connect and control Pixhawk/APM through [Mavlink](https://github.com/mavlink/mavlink/). This is a project under developing.

Instead of using RC control, we use companion computer to give commands and missions to Pixhawk, then get feedback directly. There are some tutorials on http://dev.ardupilot.com that tell you how to [Communicat with Raspberry Pi via MAVLink](http://dev.ardupilot.com/wiki/raspberry-pi-via-mavlink/), [Communicat with ODroid via MAVLink](http://dev.ardupilot.com/wiki/odroid-via-mavlink/) and other articles you may be interested in.

# Features
- Get Pixhawk system info
- Get Pixhawk parameters
- Add attribute and params listener
- Arm
- Take off
- Send NED Velocity
- Set Speed
- Go to somewhere
- Add/Update/Clear command

# How to use
- Clone to local directory.
- Connect pixhawk with your companion computer(Pi, Odroid, or your laptop).
- Set the right port and baudrate for connection in test.py. Like:
```
vehicle = APMConnector('/dev/ttyUSB0',57600)
```
- Test
```
$ python test.py
```

# The MIT License (MIT)

Copyright © 2016  [ccharlieli@live.com]

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
