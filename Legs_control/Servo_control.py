"""
this file is a part of layer 1 - execute movement layer
its meant to run supporting functions for Leg_movement (the main file of layer 1)
it includes all the functions that are needed to control the servos including the startup, and calculations
"""

from math import *
from adafruit_servokit import ServoKit
import busio
import board
import time


## main function for starting communication with the servos via i2c
# returns servos which is a 16 long array representing all the servos
# note that servos 0-2 are for leg l1, 4-6 are for leg r1, 8-10 are for leg l2, and 12-14 are for leg r2
def servo_setup():
    i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
    kit = ServoKit(channels=16, i2c=i2c_bus0)
    print('servokit started')
    return kit



## receives angles array (each angle is (theta1,theta2,theta3)) in radians from the IK_calculations
# this function than converts said angles into degrees than does the necessary conversions
# to get the actual physical servo angles necessary to move the legs into the correct position and returns said angles
def servo_angles(angles_rad):
    angles_deg = convert_angles(angles_rad)
    angles_servo = []
    for i in range(len(angles_deg)):
        (temp_1, temp_2, temp_3) = angles_deg[i]
        temp_1 = 180 - (temp_1 + 90)
        temp_2 = 180 - (temp_2 + 90)
        temp_3 = 180 - temp_3
        ##see paper for the conversion explanation
        angles_servo.append((temp_1, temp_2, temp_3))
    return angles_servo


# convert angles array form radians to degrees
def convert_angles(angles_rad):
    angles_deg = []
    for i in range(len(angles_rad)):
        (temp_1, temp_2, temp_3) = angles_rad[i]
        temp_1 = degrees(temp_1)
        temp_2 = degrees(temp_2)
        temp_3 = degrees(temp_3)
        ##math.degrees(radian) converts into degrees
        angles_deg.append((temp_1, temp_2, temp_3))
    return angles_deg
