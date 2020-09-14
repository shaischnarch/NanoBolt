from Excution.All_executions import *
from Calculations.All_calculations import legIK, servo_angles
from Helper_directory import Settings

## function used to set robot to the zero position
def zero_position():
    """not_finished = True
    while not_finished:
        for i in range(4):
            a = input("enter offset for leg: " + str(i))
            a = tuple(int(x) for x in a.split(","))
            (offset1, offset2, offset3) = a
            (theta1, theta2, theta3) = legIK(-25 + offset1, -150 + offset2, 0 + offset3)
            angles_servo = servo_angles([(theta1, theta2, theta3)], i)
            execute_movement(i, angles_servo[0])

        b = input("are you finished? ")
        if b == 'yes':
            not_finished = False
    """
    default_x = Settings.default_x
    default_y = Settings.default_y
    default_z = Settings.default_z
    for i in range(4):
        (offset1, offset2, offset3) = Settings.legs_offset[i]
        (theta1, theta2, theta3) = legIK(default_x + offset1, default_y + offset2, default_z + offset3)
        angles_servo = servo_angles([(theta1, theta2, theta3)], i)
        execute_movement(i, angles_servo[0])
    #execute_movement(0, (90, 135, 90))
    #execute_movement(1, (90, 80, 60))
    #xecute_movement(2, (90, 45, 90))
    #execute_movement(3, (90, 120, 120))
    #execute_movement(0, (90, 90, 90))
    #execute_movement(1, (90, 90, 90))
    #execute_movement(2, (90, 90, 90))
    #execute_movement(3, (90, 90, 90))
    return [(default_x, default_y, default_z), (default_x, default_y, default_z), (default_x, default_y, default_z), (default_x, default_y, default_z)]
