title: Drone with 4G

theme: jdan/cleaver-retro
style: style.css

author:
  name: Charlie
  github: CCharlieLi
  email: charlie@wiredcraft.com
  url: charlieli.cn

output: index.html
controls: true

--

# [Drone with 4G](https://wiredcraft.com/blog/drone-copter-uav-4g-network/)
## Make your drone fly farther with a 4G network connection

--

### Unmanned Aerial Vehicle (UAV)

Have you ever used a UAV? or even built one?

* [DJI](http://www.dji.com/)
* [Parrot](http://www.parrot.com)
* [3D Robotics(3DR)](https://3dr.com/)

Thanks to these companies, nearly everyone can take home a rather inexpensive UAV for aerial photography or playing with as a toy. 

--

### What made up a drone?

* **Fly control/management system**
* Battery
* Electron speed regulator
* Rotor
* Airscrew
* Radio receiver/transmitter
* GPS
* ...

It's never easy to make a drone from scratch. 

--

<center>
    <img src="http://stack.formidable.com/spectacle/dist/95649f398587a84a7e69e0493c83e88e.png" height="300">
</center>

## What's wrong with drones?

--

# The maximum control distance

--

### Problems

* The maximum distance of most civilian UAVs is less than 5 miles.
* For industry-level drones, it's around 30-50 miles.
* Lose connection, out of control.

--


<center>
    <img src="http://ww1.sinaimg.cn/mw690/7650fd75jw1f6hf63fo8uj206j05y3yo.jpg" height="300">
</center>

## Let's use 4G network to control drones

--

<center>
	<a href="http://wiredcraft.com/images/posts/drone-4g-network-1.png">
    	<img src="http://wiredcraft.com/images/posts/drone-4g-network-1.png" height="500">
    </a>
</center>

--

<center>
    <img src="http://ww2.sinaimg.cn/mw690/7650fd75jw1f6hf63e6o1j209q0663yv.jpg" height="300">
</center>

## What goes into building a 4G-connected hexacopter?

--

<center>
    <img src="http://wiredcraft.com/images/posts/drone-4g-network-2.png" height="300">
</center>

## Aircraft body: [DJI S900](http://www.dji.com/cn/product/spreading-wings-s900)

--

<center>
    <img src="http://wiredcraft.com/images/posts/drone-4g-network-3.png" height="300">
</center>

## Companion computer: [ODROID XU4](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143452239825)/[BPI-M2](http://www.hardkernel.com/main/products/prdt_info.php?g_code=G143452239825)) with an Ubuntu operating system 

--

<center>
    <img src="http://wiredcraft.com/images/posts/drone-4g-network-4.png" height="300">
</center>

## Flight control board: [Pixhawk](http://pixhawk.com/)(with APM/PX4) 

--

<center>
    <img src="http://wiredcraft.com/images/posts/drone-4g-network-6.png" height="300">
</center>

## HUAWEI E3272 4G LTE dongle 

--

<center>
    <img src="http://wiredcraft.com/images/posts/drone-4g-network-7.png" height="300">
</center>

## Nova SDS011 PM2.5 sensor 

--

<center>
    <img src="http://wiredcraft.com/images/posts/drone-4g-network-8.png" height="300">
</center>

## Matek mini power hub (DIstribution board) 

--

* 5200 mah battery (for companion computer) and 16000 mah battery (for Pixhawk and rotors)
* TTL2USB module
* Dupont lines
* Fittings of Pixhawk: GPS module, buzzer, switch, etc.

## That's pretty much of it! Woohoo!

--

<center>
	<a href="http://wiredcraft.com/images/posts/drone-4g-network-9.png">
    	<img src="http://wiredcraft.com/images/posts/drone-4g-network-9.png" height="500">
    </a>
</center>

--

### [Setting up the hexacopter](https://wiredcraft.com/blog/drone-copter-uav-4g-network/)

* Set up DJI S900 framework
* Prepare Pixhawk
* Set up Pixhawk on the S900
* Test the UAV
* Set up ODROID XU4
* [Connect the PM2.5 sensor with XU4]()
* Power the XU4 with power hub
* Connect the XU4 with Pixhawk
* [Communication between Pixhawk and the XU4]()

--

### Connect the PM2.5 sensor with XU4

Use [pySensor](https://github.com/CCharlieLi/pySensor) on ODROID XU4 to read PM2.5 data from sensor. 

```
14:59:52 PM2.5:  35
14:59:53 PM2.5:  36
14:59:54 PM2.5:  35
14:59:55 PM2.5:  35
```

--

### Communication between Pixhawk and the XU4

- Use [Mavproxy](http://dronecode.github.io/MAVProxy/html/getting_started/index.html) or [Dronekit](http://python.dronekit.io/) to establish communication.
- Mavlink: A protocol designed to communicate with Pixhawk.
- [APM connector(APMC)](https://github.com/CCharlieLi/APMC): A Dronekit based command line tool connect and give direct orders to Pixhawk.

--

### Server side

- Companion computer to send HTTP requests (POST) to the server every second with its data.
- Data could include PM2.5, location, timestamp, drone state, and more. 
- XU4 gets return messages from the server as the current set of orders and passes it to Pixhawk.
<center>
    <img src="http://wiredcraft.com/images/posts/drone-4g-network-10.png" height="300">
</center>

--

### Problems and discussion

- Network connection
- Network delay

--

## That's it!
<center>
	<a href="http://wiredcraft.com/images/posts/4g-network-drone-copter-2.jpg">
    	<img src="http://wiredcraft.com/images/posts/4g-network-drone-copter-2.jpg" height="500">
    </a>
</center>



