from Plan.Controller import *
from Helper_directory import Settings
from time import sleep
import math

# (end_point, num_of_substeps, height, substep_delay, is_changed, shut_down) = oneLegPlan(current_leg_location, is_finished_step)


# initialization
max_delay = Settings.max_delay
min_delay = Settings.min_delay
num_of_substeps = 32
heights = [0, 0, 0, 0]
substep_delay = 0
is_changed = 0
shut_down = 0
move_types = [1, 2, 1, 2] # two digits, first representing 1 - forward ,2 - backwards, 0 - not moving forwards or backwards
# second representing 1 - left, 2 - right, 0 - not moving left or right
#prev_left_cy = 0
#prev_norm_cy = 0
#prev_norm_cx = 0

##### V 1.3
## for now a speed change is sent back to the main via a change of substep_delay
def plan_movement(current_leg_locations, is_finished_step, ds4):
    global max_delay
    global min_delay
    global num_of_substeps
    global heights
    global substep_delay
    global is_changed
    global shut_down
    global move_types
#    global prev_left_cy
#    global prev_norm_cy
#    global prev_norm_cx

    end_points = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]      ### end points needs a return value if is_finished_step == false, therefor this is just a temporary placeholder that isn't used

    (left_cx, left_cy, right_cx, right_cy, buttons_pressed) = controller_call(ds4)

    if (left_cy == 0 and left_cx == 0):
        shut_down = 1
        norm_cx = 1
        norm_cy = 1
    else:
        shut_down = 0
        radius = math.sqrt(left_cx*left_cx + left_cy*left_cy)       ##### fix from jetson code!!!
        substep_delay = max_delay - ((max_delay-min_delay) / granularity)*radius
        norm_cy = left_cy / radius
        norm_cx = left_cx / radius


    ## if (prev_left_y*left_y < 0) means we are changing direction
    if (is_finished_step == True):
        # for now we are just walking

        end_z = Settings.dist_Z * abs(norm_cy)
        end_x = (-25 + Settings.dist_X) * norm_cx

        for leg_num in range(4):
            """
            if (move_types[leg_num] == 'down'):
                move_types[leg_num] = 'up'
                # (real X, real Y, real Z)
                end_points[leg_num] = (end_x, -150, end_z)
                heights[leg_num] = 35*int(left_cy >= 0)

            elif (move_types[leg_num] == 'up'):
                move_types[leg_num] = 'down'
                end_points[leg_num] = (-end_x, -150, -end_z)
                heights[leg_num] = 35*int(left_cy < 0)
            print((leg_num, current_leg_locations[leg_num], end_points[leg_num]))
            """
            if (norm_cx == 0):
                if (move_types[leg_num] == 1): ## walk forwards and backward
                    move_types[leg_num] = 2
                    end_points[leg_num] = (-25, -150, end_z)
                    heights[leg_num] = 35 * int(left_cy > 0)

                elif (move_types[leg_num] == 2): ## walk forwards and backward
                    move_types[leg_num] = 1
                    end_points[leg_num] = (-25, -150, -end_z)
                    heights[leg_num] = 35*int(left_cy < 0)

            elif (norm_cy == 0):
                if (move_types[leg_num] == 1): ## walk left and right
                    move_types[leg_num] = 2
                    end_points[leg_num] = (end_x, -150, 0)
                    heights[leg_num] = 35 * int(left_cx > 0)

                elif (move_types[leg_num] == 2): ## walk left and right
                    move_types[leg_num] = 1
                    end_points[leg_num] = (-end_x, -150, 0)
                    heights[leg_num] = 35 * int(left_cx < 0)

            else:
                pass

        is_changed = 1

    else:
        is_changed = 0


    return end_points,num_of_substeps,heights,substep_delay,is_changed,shut_down

"""     
    if(left_cy != 0):
        prev_left_cy = left_cy
    if (left_cy != 0 and left_cx != 0):
            prev_norm_cy = norm_cy
            prev_norm_cx = norm_cx
"""




















