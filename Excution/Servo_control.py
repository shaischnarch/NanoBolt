"""
this file is a part of layer 1 - execute movement layer
its meant to run supporting functions for Leg_movement (the main file of layer 1)
it includes all the functions that are needed to control the servos including the startup, and calculations
"""


from adafruit_servokit import ServoKit
import busio
import board


## main function for starting communication with the servos via i2c
# returns servos which is a 16 long array representing all the servos
# note that servos 0-2 are for leg 0, 4-6 are for leg 1, 8-10 are for leg 2, and 12-14 are for leg 3
def servo_setup():
    i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
    kit = ServoKit(channels=16, i2c=i2c_bus0)
    print('servokit started')
    return kit



####### for now not in use