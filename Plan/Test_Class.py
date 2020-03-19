"""
TEST CLASS
used to run simple tests

Test #01 - simple servo test
ment to check servo angles and connectivity
"""
from adafruit_servokit import ServoKit
from Calculations import Movement_display as display
from Calculations.All_calculations import *
import busio
import board
import time



i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))


kit = ServoKit(channels = 16, i2c = i2c_bus0)
print('servokit started')



start_point = (0,-100,0)
end_point = (50,-100,50)
angles = calculate_movement(start_point,end_point,50,8)
display.draw_movement(angles)

angles_for_servos = servo_angles(convert_angles(angles))
for i in range(len(angles_for_servos)):
		(temp_0, temp_1, temp_2) = angles_for_servos[i]
		kit.servo[0].angle = temp_0
		kit.servo[1].angle = temp_1
		kit.servo[2].angle = temp_2
		sleep(0.5)







"""
from time import sleep

from approxeng.input.dualshock4 import DualShock4
from approxeng.input.selectbinder import ControllerResource, ControllerRequirement


while True:
    hue = 0.0
    try:
        # Force waiting for a DS4 controller, as that's the only one with the call
        # to set the light bar in this way.
        with ControllerResource(ControllerRequirement(require_class=DualShock4)) as ds4:
            while ds4.connected:
                left_x,left_y = ds4['lx', 'ly']
                left_x = round(left_x*5)
                left_y = round(left_y*5)
                print(str(left_x) + '\t' + str(left_y))
                sleep(0.5)
    except IOError:
        # No DS4 controller found, wait for a bit and try again
        print('Waiting for a DS4 controller connection')
        sleep(1)
"""
