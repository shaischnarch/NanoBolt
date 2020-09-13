

## granularity used for controller interpretation
granularity = 5


## max substep delay
max_delay = 0.05



## min substep delay
min_delay = 0.0075



## step parameters:
dist_X = 20 # the max size of a single step in the X direction (millimeters)
dist_Z = 20 # the max size of a single step in the Z direction (millimeters)
base_height = -150 # the height of the robot above ground in its base position (standing position)

## the calculation for  added  step height : step_height*height_normalization(x-x^2)^height_power
step_height = 20
height_power= 0.5
height_normalization= 0.25**(-height_power)

legs_offset = [(-10,-15,25),(0,18,-15),(0,-22,-25),(0,-20,-10)] # the offset for each leg relative to regular step (legs 0,1,2,3)
#legs_offset = [(0,0,0),(0,0,0),(0,-15,-35),(0,-15,-35)]

## max absolute value of leg X value
max_X = 90

## max absolute value of leg Z value
max_Z = 90

## min value of leg Y value
min_Y = -25

## max absolute value of leg Y value
max_Y = -170
