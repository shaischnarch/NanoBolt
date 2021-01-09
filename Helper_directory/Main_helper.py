from Excution.All_executions import *
from Calculations.All_calculations import legIK, servo_angles, calculate_movement, calculate_points
from Helper_directory import Settings
import math
import time

## function used to set robot to the zero position
def zero_position():
    for i in range(4):
        (x,y,z) = Settings.semi_ideal_stand_pos[i]
        (theta1, theta2, theta3) = legIK(x,y,z)
        angles_servo = servo_angles([(theta1, theta2, theta3)], i)
        execute_movement(i, angles_servo[0])
    return Settings.semi_ideal_stand_pos[:], Settings.semi_ideal_stand_pos[:] ## returns the semi-ideal stand position (which at this point is the current_leg_location)



## Main function for moving from current location to a target position
#  The current legs location is in the physical realm, the target_position is in the semi-ideal realm.
#  This methods moves the robot to the target_position where the dynamic offsets are 0.
#  After this method, current_legs_location and semi_ideal_position should be equal
def move_to_position(target_position, current_legs_location):
    ### speed parameter in mm per max_delay
    speed = 5
    max_dist = 0
    for i in range(4):
        (target_x, target_y, target_z) = target_position[i]
        (curr_x, curr_y, curr_z) = current_legs_location[i]
        diff_x = math.fabs(target_x - curr_x)
        if (diff_x > max_dist):
            max_dist = diff_x
        y_dist_multiplier = 3
        diff_y = math.fabs(target_y - curr_y)
        diff_y = diff_y * y_dist_multiplier
        if (diff_y > max_dist):
            max_dist = diff_y
        diff_z = math.fabs(target_z - curr_z)
        if (diff_z > max_dist):
            max_dist = diff_z

    num_of_substeps = int(max_dist / speed) + 1

    angles_rad = [[], [], [], []]
    all_angles = [[], [], [], []]


    # calculates the substeps (semi-ideal) for the step from current_legs_location to target_position
    for leg_num in range(4):
        (target_x, target_y, target_z) = target_position[leg_num]
        angles_rad[leg_num] = calculate_movement(current_legs_location[leg_num], (target_x, target_y, target_z), 0, num_of_substeps)
        all_angles[leg_num] = servo_angles(angles_rad[leg_num], leg_num)

    for current_substep_num in range(num_of_substeps + 1):
        for leg_num in range(4):
            execute_movement(leg_num, all_angles[leg_num][current_substep_num])
        time.sleep(Settings.max_delay)
    print("done")
    return target_position[:], target_position[:]




## Main function for moving from current location to the robots default position (stand)
#  The current legs location is in the physical realm
#  This methods moves the robot to the standing_position where the dynamic offsets are 0.
#  After this method, current_legs_location and semi_ideal_position should be equal
def move_to_stand(current_legs_location):
    return move_to_position(Settings.semi_ideal_stand_pos[:], current_legs_location)




## Main function for removing dynamic offsets correctly
#  This method receives the current_legs_location and the offset that we want removed
#  This method moves the robot to the position without the given offsets
#  Returns the new current_legs_location
def remove_offset(current_legs_location, offset):
    ### speed parameter in mm per max_delay
    speed = 5
    max_dist = 0
    for i in range(4):
        (offset_x, offset_y, offset_z) = offset[i]
        if (offset_x > max_dist):
            max_dist = offset_x
        y_dist_multiplier = 3
        offset_y = offset_y * y_dist_multiplier
        if (offset_y > max_dist):
            max_dist = offset_y
        if (offset_z > max_dist):
            max_dist = offset_z

    num_of_substeps = int(max_dist / speed) + 1

    angles_rad = [[], [], [], []]
    all_angles = [[], [], [], []]

    # calculates the substeps (semi-ideal) for the step from current_legs_location to target_position
    target_position = []
    for leg_num in range(4):
        (offset_x, offset_y, offset_z) = offset[leg_num]
        (curr_x, curr_y, curr_z) = current_legs_location[leg_num]
        (target_x, target_y, target_z) = (curr_x - offset_x, curr_y - offset_y, curr_z - offset_z)
        target_position.append((target_x, target_y, target_z))
        angles_rad[leg_num] = calculate_movement(current_legs_location[leg_num], (target_x, target_y, target_z), 0, num_of_substeps)
        all_angles[leg_num] = servo_angles(angles_rad[leg_num], leg_num)

    for current_substep_num in range(num_of_substeps + 1):
        for leg_num in range(4):
            execute_movement(leg_num, all_angles[leg_num][current_substep_num])
        time.sleep(Settings.max_delay)

    return target_position



### Method used for reading the sensor values while accounting for sensor offsets
### USE ONLY THIS METHOD IN THE CODE, DO NOT USE sensor.euler
def read_sensor(sensor):
    (offset_1, offset_2, offset_3) = Settings.sensor_offset
    (euler1, euler2, euler3) = sensor.euler
    return (euler1+offset_1, euler2+offset_2, euler3+offset_3)



