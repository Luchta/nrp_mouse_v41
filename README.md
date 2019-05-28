# NRP_Mouse V4 - standard Servo Version - No Feedback

Control Software for remote control of the NRP Robot Mouse.

## Prerequisites

Setup a ROS environment on the RaspberryPi

Install the adafruit ServoKit Library

  https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/using-the-python-library
  

## Setup & Compile
clone this repository into your

```
~/mouse_ws/src
```

in mouse_ws call:

```
catkin_make
```

## Run
1. start Roscore
2. start Publisher Node: 
  ```
   roslaunch bkp_mouse bkp_mouse_node
  ```
3. Run subscriber Python3 script
  ```
  adafruit_sub.py
  ```
##Control
control the robot via the keyboard:
i: inital pose
w: forward trott
a: left
d: right
q: close programm
