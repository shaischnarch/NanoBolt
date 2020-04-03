from Plan.Controller import *
from Helper_directory import Settings
from time import sleep
import math

# (end_point, num_of_substeps, height, substep_delay, is_changed, shut_down) = oneLegPlan(current_leg_location, is_finished_step)


# initialization
max_delay = Settings.max_delay
min_delay = Settings.min_delay
num_of_substeps = 32
height = 0
substep_delay = 0
is_changed = 0
shut_down = 0
move_type = 'down'
#prev_left_cy = 0
#prev_norm_cy = 0
#prev_norm_cx = 0


"""
### V 1.0

def plan_movement(current_leg_location, is_finished_step):
    end_point = current_leg_location
    global num_of_substeps
    global height
    global substep_delay
    global is_changed
    global shut_down
    global type  # type = [forward, backward, sit, stand, plank]
    global speed  # speed = [slow, fast]
    ps4_signal ## 0 means no signal was received 1 means circle was received


    # check if there was a new cmd from the remote control - change type and speed accordingly
    ### if received signal ps4_signal = 1
    # check if shut down - break while loop and exit

    if(ps4_signal == 1):
        end_point = (0, -150, 0)
        height = 0
        num_of_substeps = 8
        substep_delay = 0.02  # seconds
        is_changed = 1
        shut_down = 0

    elif (is_finished_step == True):

        if (speed == 'slow'):
            num_of_substeps = 64
            substep_delay = 0.02 #seconds
        else:
            num_of_substeps = 32
            substep_delay = 0.0225 #seconds

        if (type == 'forward'):
            if (current_leg_location[2] == -60):
                end_point = (-25,-150,60)
                height = 35
            elif (current_leg_location[2] == 60):
                end_point = (-25,-150,-60)
                height = 0
            is_changed = 1
            shut_down = 0
        elif (type == 'backward'):
            if (current_leg_location[2] == 60):
                end_point = (-25,-150,-60)
                height = 35
            elif (current_leg_location[2] == -60):
                end_point = (-25,-150,60)
                height = 0
            is_changed = 1
            shut_down = 0

    else:
        is_changed = 0

    return end_point,num_of_substeps,height,substep_delay,is_changed,shut_down


##### V 1.1
## for now a speed change is sent back to the main via a change of substep_delay
def plan_movement(current_leg_location, is_finished_step, ds4):
    global max_delay
    global min_delay
    global num_of_substeps
    global height
    global substep_delay
    global is_changed
    global shut_down

    end_point = current_leg_location

    (left_x, left_y, right_x, right_y, buttons_pressed) = controller_call(ds4)

    if (left_y <= 0):
        shut_down = 1
    else:
        shut_down = 0
        substep_delay = max_delay - ((max_delay-min_delay) / granularity)*left_y



    if (is_finished_step == True):

        if (current_leg_location[2] == -75):
            end_point = (-25, -150, 75)
            height = 35
        elif (current_leg_location[2] == 75):
            end_point = (-25, -150, -75)
            height = 0
        is_changed = 1


    else:
        is_changed = 0

    return end_point,num_of_substeps,height,substep_delay,is_changed,shut_down
"""


##### V 1.2
## for now a speed change is sent back to the main via a change of substep_delay
def plan_movement(current_leg_location, is_finished_step, ds4):
    global max_delay
    global min_delay
    global num_of_substeps
    global height
    global substep_delay
    global is_changed
    global shut_down
    global move_type
#    global prev_left_cy
#    global prev_norm_cy
#    global prev_norm_cx

    end_point = current_leg_location

    (left_cx, left_cy, right_cx, right_cy, buttons_pressed) = controller_call(ds4)

    if (left_cy == 0 and left_cx == 0):
        shut_down = 1
        norm_cx = 1
        norm_cy = 1
    else:
        shut_down = 0
        radius = math.sqrt(left_cx*left_cx + left_cy*left_cy)
        substep_delay = max_delay - ((max_delay-min_delay) / granularity)*radius
        norm_cy = left_cy / radius
        norm_cx = left_cx / radius


    ## if (prev_left_y*left_y < 0) means we are changing direction
    if (is_finished_step == True):

        end_z = Settings.dist_Z * abs(norm_cy)
        end_x = (-25 + Settings.dist_X) * norm_cx

        if (move_type == 'down'):
            move_type = 'up'


            # (real X, real Y, real Z)
            end_point = (end_x, -150, end_z)
            height = 35*int(left_cy > 0)

        elif (move_type == 'up'):
            move_type = 'down'
            end_point = (-end_x, -150, -end_z)
            height = 35*int(left_cy < 0)
        is_changed = 1


    else:
        is_changed = 0


    return end_point,num_of_substeps,height,substep_delay,is_changed,shut_down

"""     
    if(left_cy != 0):
        prev_left_cy = left_cy
    if (left_cy != 0 and left_cx != 0):
            prev_norm_cy = norm_cy
            prev_norm_cx = norm_cx
"""




















