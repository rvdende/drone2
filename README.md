drone
=====

DIY Arduino DUE (ARM 32bit) quad copter drone with node.js telemetry
Aimed to be flexible enough that anyone can jump in and use their own hardware.

WARNING
=======

Flying robots are dangerous, and can be unpredictable, cut eyes and limbs. Test without blades on your motors, and secure your drone down while testing. Good luck!

Status
=======

2  November  2013 - Receiver support added and motor output simplified.
18 October   2013 - Sampling rate up to 1160hz! Reduced noise and responsiveness.
09 October   2013 - Vector gyro/compass/accelerometer reference system completed
27 September 2013 - Started

Documentation
=============

Set your arduino ide to the main folder so it finds the `/libraries` folder, alternatively copy them over to your libraries folder. Expand them to support your hardware.

Load up `/drone/drone.ino` in Arduino IDE and have fun.

See `/drone/UserConfiguration.h` for quick settings. 

See `/libraries` for motor and receiver PINs.

Channel 5 on the receiver arms the PID system. You can see LED on pin 13 will be ON when it's armed.

More detailed documentation to follow. 

A lot of source is from `https://github.com/AeroQuad/AeroQuad` 

Hardware
=========

[MinIMU-9 v2 Gyro, Accelerometer, and Compass (L3GD20 and LSM303DLHC Carrier)](http://www.pololu.com/catalog/product/1268)

[Barometric Pressure Sensor - BMP085 Breakout](https://www.sparkfun.com/products/11282)

[Adafruit Ultimate GPS Logger Shield](http://www.adafruit.com/products/1272)

![photo!](https://raw.github.com/fluentart/drone2/master/photo.jpg)

Telemetry
===========

droneConnect gives you a realtime 3D in the browser vector representation of what the drone sees, useful for debugging and developing further. Also included is a realtime 2D graph view showing realtime telemetry from the different axis and motor throttles. It also allows you to set the PID feedback loop calibration settings for the stabilisation of the drone.

![axis.jpg!](https://raw.github.com/fluentart/drone2/master/droneConnect/static/img/axis.jpg) ![graph.jpg!](https://raw.github.com/fluentart/drone2/master/droneConnect/static/img/graph.jpg)

Open up command prompt.

`cd drone\droneConnect`

You probably have to edit the droneConnect.js file and specify the correct COM port. Then run:

`node droneConnect.js`

Open up [localhost/graph.htm](http://localhost/) and enjoy.
