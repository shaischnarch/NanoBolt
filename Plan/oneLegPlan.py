# (end_point, num_of_substeps, height, substep_delay, is_changed, shut_down) = oneLegPlan(current_leg_location, is_finished_step)


# initialization
num_of_substeps = 0
height = 0
substep_delay = 0
is_changed = 0
shut_down = 0
type = 'forward'  # type = [forward, backward, sit, stand, plank]
speed = 'fast'  # speed = [slow, fast]
ps4_signal = 0 ## 0 means no signal was received 1 means circle was received

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
