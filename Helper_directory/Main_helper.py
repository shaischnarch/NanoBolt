from Excution.All_executions import *
from Calculations.All_calculations import legIK, servo_angles , calculate_movement
from Helper_directory import Settings
import math

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
    #return [(default_x, default_y, default_z), (default_x, default_y, default_z), (default_x, default_y, default_z), (default_x, default_y, default_z)]
    return Settings.default_with_offset[:] ## return the default with offest insted the default

def move_to_stand(current_legs_location):

    ### speed parameter in mm per max_delay
    speed = 5

    default_with_offset = Settings.default_with_offset[:]
    max_dist = 0
    for i in range(4):
        (temp_x, temp_y, temp_z) = default_with_offset[i]
        (curr_x, curr_y, curr_z) = current_legs_location[i]
        diff_x = math.fabs(temp_x - curr_x)
        if (diff_x > max_dist):
            max_dist = diff_x
        y_dist_multiplier = 3
        diff_y = math.fabs(temp_y - curr_y)
        diff_y = diff_y * y_dist_multiplier
        if (diff_y > max_dist):
            max_dist = diff_y
        diff_z = math.fabs(temp_z - curr_z)
        if (diff_z > max_dist):
            max_dist = diff_z

    num_of_substeps = int(max_dist/speed) + 1

    angles_rad = [[], [], [], []]
    all_angles = [[], [], [], []]

    for leg_num in range(4):
        (temp_x, temp_y, temp_z) = default_with_offset[i]
        angles_rad[leg_num] = calculate_movement(current_legs_location[leg_num], (temp_x, temp_y, temp_z), 0, num_of_substeps) #### make sure that calculate movement is designed for both left and right
        all_angles[leg_num] = servo_angles(angles_rad[leg_num], leg_num)


    for current_substep_num in range(num_of_substeps+1):
        for leg_num in range(4):
            execute_movement(leg_num, all_angles[leg_num][current_substep_num])
        time.sleep(Settings.max_delay)


    return default_with_offset
