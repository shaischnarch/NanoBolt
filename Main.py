from Calculations.All_calculations import *
from Excution.All_executions import *
from Plan.oneLegPlan import *
from approxeng.input.dualshock4 import DualShock4
from approxeng.input.selectbinder import ControllerResource, ControllerRequirement
from Helper_directory.Main_helper import *
import time

## initialization
servo_setup()  # starts i2c communication with servos
current_leg_locations = zero_position()
sleep(1)  # gives enough time to get to zero position



points = [[], [], [], []]
is_finished_step = True
all_angles = [[], [], [], []]
current_substep_num = 0
last_substep_num = 0
####current_substep_num = [0, 0, 0, 0]
####last_substep_num = [0, 0, 0, 0]


## here starts communication with ps4 controller
while True:

	try:
		with ControllerResource(ControllerRequirement(require_class=DualShock4)) as ds4:
			while ds4.connected:
				print('Controller Connected')
				# main loop
				while (ds4.connected):
					(end_points, num_of_substeps, heights, substep_delay, is_changed, shut_down) = plan_movement(current_leg_locations, is_finished_step, ds4)
					### for now just tests pausing movement
					if (shut_down == True):
						sleep(substep_delay)
						continue


					if (is_changed == True):
						angles_rad = [[], [], [], []]
						is_finished_step = False
						current_substep_num = 0
						last_substep_num = num_of_substeps
						# calculate_movement(start_p, end_p, height, num_of_substeps):
						# all_angles should be in servo-angles
						for leg_num in range(4):
							print((leg_num, current_leg_locations[leg_num], end_points[leg_num], num_of_substeps, heights[leg_num], substep_delay, is_changed, shut_down))
							angles_rad[leg_num] = calculate_movement(current_leg_locations[leg_num], end_points[leg_num], heights[leg_num], num_of_substeps) #### make sure that calculate movement is designed for both left and right
							all_angles[leg_num] = servo_angles(angles_rad[leg_num], leg_num)
							points[leg_num] = calculate_points(current_leg_locations[leg_num], end_points[leg_num], heights[leg_num], num_of_substeps) ###same as calculate movement

					for leg_num in range(4):
						execute_movement(leg_num, all_angles[leg_num][current_substep_num])
						current_leg_locations[leg_num] = points[leg_num][current_substep_num]
					current_substep_num += 1


					if (current_substep_num >= last_substep_num):
						is_finished_step = True
					# current_substep_num -= 1 ## this is here in case plan failed and is_changed == false

					time.sleep(substep_delay)


				######### this is out of the while(1). Shutdown everything ########################

	except IOError:
		## set the robot to the starting zero position
		zero_position()
		current_leg_locations = zero_position()
		# No DS4 controller found, wait for a bit and try again
		print('Waiting for a DS4 controller connection')
		sleep(1)  # temp

