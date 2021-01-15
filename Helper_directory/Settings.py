

## granularity used for controller interpretation
granularity = 5


## max substep delay in seconds
max_delay = 0.05



## min substep delay in seconds
min_delay = 0.0075




## step parameters:
dist_X = 20  # the max size of a single step in the X direction (millimeters)
dist_Z = 20  # the max size of a single step in the Z direction (millimeters)
base_height = -150  # the height of the robot above ground in its base position (standing position)

## the calculation for  added  step height : step_height*height_normalization(x-x^2)^height_power
step_height = 20
height_power = 0.5
height_normalization = 0.25**(-height_power)



## default values zero position
__default_x = -25
__default_y = -135
__default_z = 0


__legs_offset = [(-12,-15,9),(0,-10,11),(0,-14,-35),(13,-16,-25)]  # the offset for each leg relative to regular step (legs 0,1,2,3)
#legs_offset = [(0,0,0),(0,0,0),(0,-15,-35),(0,-15,-35)]

sensor_offset = (0,0,2)  # when standing level (measured using a level) the values of the sensor where (-, 0, -2)

## Calculates the ideal position from the standing position with an offset
def __offset_from_stand(offset):
    retVal = [0, 0, 0, 0]
    for i in range(4):
        (offset_x, offset_y, offset_z) = offset[i]
        retVal[i] = (__default_x + offset_x, __default_y + offset_y, __default_z + offset_z)
    return retVal



## Calculates the semi-ideal position from the ideal position
def __ideal_to_semiIdeal(ideal_pos):
    retVal = [0, 0, 0, 0]
    for i in range(4):
        (ideal_x, ideal_y, ideal_z) = ideal_pos[i]
        (leg_offset_x, leg_offset_y, leg_offset_z) = __legs_offset[i]
        retVal[i] = (leg_offset_x + ideal_x, leg_offset_y + ideal_y, leg_offset_z + ideal_z)
    return retVal





#the default values combined with the offset for each leg
__stand_leg_position = (__default_x,__default_y, __default_z)
semi_ideal_stand_pos = __ideal_to_semiIdeal([__stand_leg_position, __stand_leg_position, __stand_leg_position, __stand_leg_position])



## Stable 3 legs:
__stable_3_legs_right_offset = [(40,0,10), (0,50,0), (-15,0,10), (0,10,0)]  # offset from stand
__ideal_stable_3_legs_right = __offset_from_stand(__stable_3_legs_right_offset)  # ideal default position
stable_3_legs_right_default = __ideal_to_semiIdeal(__ideal_stable_3_legs_right)  # semi-ideal default position

__stable_3_legs_left_offset = [(0,50,0), (40,0,10), (0,10,0), (-15,0,10)]  # offset from stand
__ideal_stable_3_legs_left = __offset_from_stand(__stable_3_legs_left_offset)  # ideal default position
stable_3_legs_left_default = __ideal_to_semiIdeal(__ideal_stable_3_legs_left)  # semi-ideal default position



### TEST TODO: implement or delete ###
## Walking 2 legs:
#forwards: When walking forwards in a straight line
__first_diag_forward_offset = [(0,0,17), (0,0,-33), (0,0,17), (0,0,-33)]  # first diag is legs 0,2 in the forward position
__ideal_first_diag_forward = __offset_from_stand(__first_diag_forward_offset)
first_diag_forward_default = __ideal_to_semiIdeal(__ideal_first_diag_forward)

__second_diag_forward_offset = [(0,0,-33), (0,0,17), (0,0,-33), (0,0,17)]  # second diag is legs 1,3 in the forward position
__ideal_second_diag_forward = __offset_from_stand(__second_diag_forward_offset)
second_diag_forward_default = __ideal_to_semiIdeal(__ideal_second_diag_forward)

#backwards: when walking backwards in a straight line
#__first_diag_backward_offset = [(0,0,-25), (0,0,25), (0,0,-25), (0,0,25)]  # first diag is legs 0,2 in the forward position
__first_diag_backward_offset = [(0,0,-20), (0,0,20), (0,0,-20), (0,0,20)]  # first diag is legs 0,2 in the forward position
__ideal_first_diag_backward = __offset_from_stand(__first_diag_backward_offset)
first_diag_backward_default = __ideal_to_semiIdeal(__ideal_first_diag_backward)

#__second_diag_backward_offset = [(0,0,25), (0,0,-25), (0,0,25), (0,0,-25)]  # second diag is legs 1,3 in the forward position
__second_diag_backward_offset = [(0,0,20), (0,0,-20), (0,0,20), (0,0,-20)]  # second diag is legs 1,3 in the forward position
__ideal_second_diag_backward = __offset_from_stand(__second_diag_backward_offset)
second_diag_backward_default = __ideal_to_semiIdeal(__ideal_second_diag_backward)

## max absolute value of leg X value
max_X = 90

## max absolute value of leg Z value
max_Z = 90

## min value of leg Y value
min_Y = -25

## max absolute value of leg Y value
max_Y = -170


