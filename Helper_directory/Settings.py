

## granularity used for controller interpretation
granularity = 5


## max substep delay
max_delay = 0.05



## min substep delay
min_delay = 0.0075


## default values zero position
default_x = -25
default_y = -135
default_z = 0

## step parameters:
dist_X = 20 # the max size of a single step in the X direction (millimeters)
dist_Z = 20 # the max size of a single step in the Z direction (millimeters)
base_height = -150 # the height of the robot above ground in its base position (standing position)

## the calculation for  added  step height : step_height*height_normalization(x-x^2)^height_power
step_height = 20
height_power= 0.5
height_normalization= 0.25**(-height_power)

legs_offset = [(-20,-12,10),(5,-8,10),(2,-10,-23),(2,-12,-8)] # the offset for each leg relative to regular step (legs 0,1,2,3)
#legs_offset = [(0,0,0),(0,0,0),(0,-15,-35),(0,-15,-35)]

#the default values combined with the offset for each leg
default_with_offset = []
for i in range(4):
    (offset1, offset2, offset3) = legs_offset[i]
    default_with_offset[i] = (default_x + offset1, default_y + offset2, default_z + offset3)




## max absolute value of leg X value
max_X = 90

## max absolute value of leg Z value
max_Z = 90

## min value of leg Y value
min_Y = -25

## max absolute value of leg Y value
max_Y = -170
