from Calculations.All_calculations import *
from Excution.All_executions import *
from Plan.oneLegPlan import *
from approxeng.input.dualshock4 import DualShock4
from approxeng.input.selectbinder import ControllerResource, ControllerRequirement
from Helper_directory.Main_helper import *
import time



## initialization
servo_setup()  # starts i2c communication with servos
zero_position()
sleep(1)

points = []
is_finished_step = True
current_leg_location = (-25,-150,60)
all_angles = []
current_substep_num = 0
last_substep_num = 0


## here starts communication with ps4 controller
while True:

	try:
		with ControllerResource(ControllerRequirement(require_class=DualShock4)) as ds4:
			while ds4.connected:
				print('Controller Connected')

				# main loop
				while (1):

					(end_point, num_of_substeps, height, substep_delay, is_changed, shut_down) = plan_movement(current_leg_location, is_finished_step, ds4)

					### for now just tests pausing movement
					if (shut_down == True):
						sleep(substep_delay)
						continue

					if (is_changed == True):
						print((current_leg_location, end_point, num_of_substeps, height, substep_delay, is_changed, shut_down))
						is_finished_step = False
						current_substep_num = 0
						last_substep_num = num_of_substeps
						# calculate_movement(start_p, end_p, height, num_of_substeps):
						# all_angles should be in servo-angles
						angles_rad = calculate_movement(current_leg_location, end_point, height, num_of_substeps)
						all_angles = servo_angles(angles_rad, 'left')
						points = calculate_points(current_leg_location, end_point, height, num_of_substeps)

					execute_movement(0, all_angles[current_substep_num])
					current_substep_num += 1
					current_leg_location = points[current_substep_num]

					if (current_substep_num >= last_substep_num):
						is_finished_step = True
					# current_substep_num -= 1 ## this is here in case plan failed and is_changed == false

					time.sleep(substep_delay)

				######### this is out of the while(1). Shutdown everything ########################

	except IOError:
		## set the robot to the starting zero position
		zero_positon()

		# No DS4 controller found, wait for a bit and try again
		print('Waiting for a DS4 controller connection')
		sleep(1)  # temp



""""

# main loop
while (1):

	(end_point, num_of_substeps, height, substep_delay, is_changed, shut_down) = plan_movement(current_leg_location, is_finished_step)

	### for now just tests pausing movement
	if (shut_down == True):
		sleep(substep_delay)
		continue

	if (is_changed == True):
		print((current_leg_location,end_point, num_of_substeps, height, substep_delay, is_changed, shut_down))
		is_finished_step = False
		current_substep_num = 0
		last_substep_num = num_of_substeps
            	# calculate_movement(start_p, end_p, height, num_of_substeps):
            	# all_angles should be in servo-angles
		angles_rad = calculate_movement(current_leg_location, end_point, height, num_of_substeps)
		all_angles = servo_angles(angles_rad, 'left')
		points = calculate_points(current_leg_location, end_point, height, num_of_substeps)

	execute_movement(0, all_angles[current_substep_num])
	current_substep_num += 1
	current_leg_location = points[current_substep_num]

	if (current_substep_num >= last_substep_num):
		is_finished_step = True
           	#current_substep_num -= 1 ## this is here in case plan failed and is_changed == false

	time.sleep(substep_delay)

    ######### this is out of the while(1). Shutdown everything ########################
"""