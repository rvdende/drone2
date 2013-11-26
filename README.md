drone
=====

DIY Arduino DUE (ARM 32bit) quad copter drone with node.js telemetry. The arduino recieves controller signals from your hobby RC remote and mixes in gyro compensated motor commands to help steer the drone.
Aimed to be flexible enough that anyone can jump in and use their own hardware.



![!drone](http://i.imgur.com/pnxbKG5.jpg)



WARNING
=======

Flying robots are dangerous, and can be unpredictable, cut eyes and limbs. Test without blades on your motors, and secure your drone down while testing. Good luck!

Status
=======

    6  November  2013 - Release 1.0.0! With 5 successful flights in 2 days.
    2  November  2013 - Receiver support added and motor output simplified.
    18 October   2013 - Sampling rate up to 1160hz! Reduced noise and increased responsiveness.
    09 October   2013 - Vector gyro/compass/accelerometer reference system completed
    27 September 2013 - Started

Documentation
=============

See hardware below.

Set your arduino ide to the main folder so it finds the `/libraries` folder, alternatively copy them over to your libraries folder. Expand them to support your hardware.

Load up `/drone/drone.ino` in Arduino IDE and have fun.

See `/drone/UserConfiguration.h` for quick settings. 

See `/libraries` for motor and receiver PINs.

Channel 5 on the receiver arms the PID system. What you want to do is throttle up slowly with it turned off just up to the point where you are ready to lift off, then flip the ch5 switch to arm the PID to help you take off and fly. If you stall too long on the ground the computer will try to fight the ground over time, so commit to it and launch after flipping on the switch.

We've now got 3 status LEDS: RED, YELLOW GREEN. on pins 13, 12, 11. This will give you more of an idea whats going on with the drone before flying. RED is on while its loading, with YELLOW indicating progress with blinks. After initialization and calibration of sensors you'll have only GREEN and YELLOW on till reciever signal is good. Then only GREEN. Switching on `armed = 1` mode with channel5 (by default) will light up all 3 LEDs with fast sporadic blinking.

First balance your drone with only 2 opposite side motors, till it can achieve balance without too much wobbling. Then the other two ofcourse same thing. Then you want to suspend the drone with rope so you can test the direction stabilisation, it should fight you noticably to maintain its direction. Be sure to also check for noise on your sensors, vibration on a loose gyro, or electrical/magentic field noise on the compass can cause problems, test with the droneConnect tool over USB serial while the drone is securely tied down. Give it quite some throttle and see that everything looks in manageable range. It shouldnt affect your motor throttles to the point where it becomes unstable. Compass direction is a problem with large motors and metal frames, so be aware.

More detailed documentation to follow shortly including pinout diagram. Alternatively have a look in libraries for the motor control and receiver code to see or change pinouts.

A lot of source is from https://github.com/AeroQuad/AeroQuad big thanks goes out to https://github.com/kh4 for DUE reciver and motor code.

Hardware recommendations
========================

[Arduino DUE](http://arduino.cc/en/Main/ArduinoBoardDue) This will be the flight controller, sitting inbetween your reciever and your speed controllers. Available in South Africa from http://robotics.org.za/index.php?route=product/product&path=47&product_id=604

[MinIMU-9 v2 Gyro, Accelerometer, and Compass (L3GD20 and LSM303DLHC Carrier)](http://www.pololu.com/catalog/product/1268) I'm using the MiniIMU-9 for gyro/accel/compass. You can add support for your sensor.

Speedcontrollers with low latency. I'm using 4x Hobbywing Skywalker 40A flashed with the blheli firmware. See [BLHELI](https://github.com/bitdump/BLHeli) and [Flashing BLHELI](http://www.olliw.eu/2012/owsilprog-tutorials/?en)

10mm or 12mm square aluminium tubing for the arms. 

Aluminium plate for the center battery plate.

2x copperclad circuit board, laser printer and ferric chloride, if you want to make a power distribution board which doubles as structural support. Circuit to be posted soon.

Velcro straps to mount battery.

Low-voltage buzzer for warning you to stop flying when LiPo battery is getting too low.

3300mah 4cell LiPo battery or you can go 3cell too.

4x EMAX 2815/05 motors 1500kv

4x 8" props. Two right turn, two left turn. (Get more props if possible, you will break them).

![photo!](https://raw.github.com/fluentart/drone2/master/photo.jpg)

PC Connect
===========

droneConnect gives you a realtime 3D in the browser vector representation of what the drone sees, useful for debugging and developing further. Also included is a realtime 2D graph view showing realtime telemetry from the different axis and motor throttles. It also allows you to set the PID feedback loop calibration settings for the stabilisation of the drone.

![axis.jpg!](https://raw.github.com/fluentart/drone2/master/droneConnect/static/img/axis.jpg) ![graph.jpg!](https://raw.github.com/fluentart/drone2/master/droneConnect/static/img/graph.jpg)

Open up command prompt.

`cd drone\droneConnect`

You probably have to edit the droneConnect.js file and specify the correct COM port. Then run:

`node droneConnect.js`

Open up [localhost/graph.htm](http://localhost/) and enjoy.

![!drone](http://i.imgur.com/qgPksgo.jpg)

drone 1 prototype
