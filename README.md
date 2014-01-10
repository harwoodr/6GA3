6GA3
====

3 slide pot, acceleromter based, self correcting platform

sched contains the xenomai/wiringPi based real-time code for the raspberry pi to control the arduino side via I2C.

arduino contains the three sketches for the arduino side

Libraries used:

Arduino:

MMA7361 Accelerometer:
https://code.google.com/p/mma7361-library/

TLC5940 PWM Driver:
http://playground.arduino.cc/learning/TLC5940

I2C Wire Library:
http://arduino.cc/en/reference/wire

Raspberry Pi:

Xenoma Real-Time Framework for Linux
http://www.xenomai.org/

Wiring Pi - GPIO library (including I2C)
http://wiringpi.com/
