'''
LAYER 1 - Execute movement
this layer uses the Calculations and inverse Calculations of layer 0 and executes those calculations into actual
physical movements of the legs
'''

from adafruit_servokit import ServoKit
import busio
import board

global kit

## main function for starting communication with the servos via i2c
# returns servos which is a 16 long array representing all the servos
# note that servos 0-2 are for leg 0, 4-6 are for leg 1, 8-10 are for leg 2, and 12-14 are for leg 3
def servo_setup():
    global kit
    i2c_bus0 = (busio.I2C(board.SCL_1, board.SDA_1))
    kit = ServoKit(channels=16, i2c=i2c_bus0)
    print('servokit started')




### main movement execution function
# receives which leg to move one substep and the angles for set substep and executes the movement
def execute_movement(leg_num, angles):
    global kit
    (theta1, theta2, theta3) = angles
    offset = leg_num*4
    kit.servo[offset].angle = theta1
    kit.servo[offset + 1].angle = theta2
    kit.servo[offset + 2].angle = theta3


"""

#l1,l2,r1,r2    leg globals

start_point = (-50, -140, -50)
end_point = (50, -160, 50)

def Move_leg_to(start_point, end_point, height = 30, num_of_substeps = 64, substep_delay = 0.015):

    angles_rad = calculate_movement(start_point, end_point, height, num_of_substeps)
    display.draw_movement(angles_rad)

    angles_for_servos = servo_angles(angles_rad)
    for i in range(len(angles_for_servos)):
        (temp_0, temp_1, temp_2) = angles_for_servos[i]
        kit.servo[0].angle = temp_0
        kit.servo[1].angle = temp_1
        kit.servo[2].angle = temp_2
        if (i == 0):
            time.sleep(1.5)
        time.sleep(substep_delay)
"""