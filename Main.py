from Calculations.All_calculations import *
from Excution.All_executions import *
from Plan import *
import time


def main():
    ## initialization
    # stand --> current_leg_locations[4], is_finished_step[4]

    ###### temporary ######
    is_finished_step = [True, True, True, True]
    current_leg_locations = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
    all_angles = [[0], [0], [0], [0]]
    current_substep_num = [0, 0, 0, 0]
    last_substep_num = [0, 0, 0, 0]


    # main loop
    while (1):
        (end_points, num_of_substeps, height, substep_delay, is_changed, shut_down) = plan_movement(current_leg_locations, is_finished_step)
        if (shut_down == True):
            break

        for leg_num in range(4):
            if (is_changed[leg_num] == True):
                is_finished_step[leg_num] = False
                current_substep_num[leg_num] = 0
                last_substep_num[leg_num] = num_of_substeps
                # calculate_movement(start_p, end_p, height, num_of_substeps):
                all_angles[leg_num] = calculate_movement(current_leg_locations[leg_num], end_points[leg_num], height[leg_num], num_of_substeps[leg_num])
            execute_movement(leg_num, all_angles[leg_num][current_substep_num])
            current_substep_num[leg_num] += 1
            if (current_substep_num[leg_num] >= last_substep_num[leg_num]):
                is_finished_step[leg_num] = True
                current_substep_num[leg_num] -= 1 ## this is here in case plan failed and is_changed == false

        time.sleep(substep_delay)

    ######### this is out of the while(1). Shutdown everything ########################










"""
main:
	#initialization
	
	#main robot code
	while(1):
	{
		#we run the plan block of code and send it current_leg_locations and if a leg finished all its substeps
		#plan_returns is a struct that the plan returns. its fields are: (end_points[4],num_of_substeps[4],height[4]
		# substep_delay, is_changed[4], shut_down)
		plan_returns = plan(current_leg_locations[4], is_finished_step[4])
		if(plan_returns.shut_down == TRUE)
			break
		for (leg_num = 0; leg_num<4; leg_num++)
		{
			if(plan_returns.is_changed[leg_num])
			{
				is_finished_step = FALSE
				angles = calculate_movement(plan_returns(whatever is needed)[leg_num])
			}
			#execute next substep, ( angles(theta1,theta2,theta3) is the next substep) ## need to add substep counter to know which one to run
			execute_movement(leg_num, angles(theta1,theta2,theta3))
			if(last_substep)
				is_finished_step[leg_num] = TRUE
		}
		time.sleep(substep_delay)
	}	
	
	#shutdown
"""