#!/usr/bin/env python

import time  
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)


# Print servo settings
print (" ")
print ("*** Servo sweeping ***")
print (" ")

kit.servo[10].actuation_range = 180
kit.servo[10].set_pulse_width_range(600, 2300)

try:
    # Sweeps the servo motor forever
    while True:
        # From 0 to 180 degrees
        for angle in range(0,180):
            kit.servo[10].angle = angle
            time.sleep(0.005)

        # From 180 to 0 degrees
        for angle in range(180,-1,-1):
            kit.servo[10].angle = angle
            time.sleep(0.005)

except KeyboardInterrupt:
	print ("Sweep ended.")

#	kit.servo[10].angle = clamp(msgarr.data[3]) #fra1
#	kit.servo[11].angle = clamp(msgarr.data[4]) #fra2

