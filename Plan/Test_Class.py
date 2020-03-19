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







