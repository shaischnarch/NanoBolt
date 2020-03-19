from time import sleep

from approxeng.input.dualshock4 import DualShock4
from approxeng.input.selectbinder import ControllerResource, ControllerRequirement

granularity = 5

## controller_call():
# main function communicating with the ps4 controller
# makes sure a connection is established and if it is returns the values of the axes and pressed
# buttons on the controller
# returns: (is_connected, left_x, left_y, right_x, right_y, buttons_pressed)
# is_connected: 0 = false    1 = true
# analog sticks values are multiplied and rounded for easy use
"""
def controller_call():
    global granularity
    sleep(0.5)  # temp
    try:
        with ControllerResource(ControllerRequirement(require_class=DualShock4)) as ds4:
            while ds4.connected:
                is_connected = 1
                left_x, left_y, right_x, right_y = ds4['lx', 'ly', 'rx', 'ry']
                buttons_pressed = ds4.check_presses()
                left_x = round(left_x*granularity)
                left_y = round(left_y*granularity)
                right_x = round(right_x * granularity)
                right_y = round(right_y * granularity)
                print('left_x = ' + str(left_x) + '\tleft_y = ' + str(left_y) + '\tright_x = ' + str(right_x) + '\tright_y = ' + str(right_y) + '\tbutton_pressed = ' + buttons_pressed)
                return (is_connected, left_x, left_y, right_x, right_y, buttons_pressed)

    except IOError:
        is_connected = 0
        # No DS4 controller found, wait for a bit and try again
        print('Waiting for a DS4 controller connection')
        return (is_connected, 0, 0, 0, 0, [])
"""
#####TEMP####
while True:
    
    try:
        with ControllerResource(ControllerRequirement(require_class=DualShock4)) as ds4:
            while ds4.connected:
                is_connected = 1
                left_x, left_y, right_x, right_y = ds4['lx', 'ly', 'rx', 'ry']
                buttons_pressed = ds4.check_presses()
                left_x = round(left_x*granularity)
                left_y = round(left_y*granularity)
                right_x = round(right_x * granularity)
                right_y = round(right_y * granularity)
                print('left_x = ' + str(left_x) + '\tleft_y = ' + str(left_y) + '\tright_x = ' + str(right_x) + '\tright_y = ' + str(right_y))
                print(buttons_pressed)
                sleep(0.5)  # temp

    except IOError:
        is_connected = 0
        # No DS4 controller found, wait for a bit and try again
        print('Waiting for a DS4 controller connection')
        sleep(1)  # temp
