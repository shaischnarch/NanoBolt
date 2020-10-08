from Calculations.All_calculations import *
from Excution.All_executions import *
from Plan.All_Plans import *
from Plan import Modes
from approxeng.input.dualshock4 import DualShock4
from approxeng.input.selectbinder import ControllerResource, ControllerRequirement
from Helper_directory.Main_helper import *
from Helper_directory import Settings
import digitalio
import time
import board
import busio
import adafruit_bno055
import math




###########################################################
# Led Pin Definitions
led = digitalio.DigitalInOut(board.D18)
led.direction = digitalio.Direction.OUTPUT

# IMU SETUP AND Definitions
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

## initialization
servo_setup()  # starts i2c communication with servos




#points = [[], [], [], []]
#is_finished_step = True
#all_angles = [[], [], [], []]
#current_substep_num = 0
#last_substep_num = 0
####current_substep_num = [0, 0, 0, 0]
####last_substep_num = [0, 0, 0, 0]
#is_stand = True
#is_2_leg = False
#two_leg_lean = True ## True means legs 0 and 2 are touching the ground, False means legs 1 and 3 are touching the ground
#two_leg_path = [[], [], [], []]
#two_leg_offset = 40
#two_leg_substeps = 80
#two_leg_counter = 0


current_leg_locations = zero_position()
sleep(1)  # gives enough time to get to zero position

###
current_mode = Stable_4_legs(current_leg_locations)

## here starts communication with ps4 controller
while True:
	led.value = True
	time.sleep(0.5)
	led.value = False
	try:
		with ControllerResource(ControllerRequirement(require_class=DualShock4)) as ds4:
			while ds4.connected:
				#turn led on
				led.value = True
				print('Controller Connected')
				#sensor_offset = [(0,0,0), (0,0,0), (0,0,0), (0,0,0)]
				# main loop
				while (ds4.connected):
					(left_cx, left_cy, right_cx, right_cy, buttons_pressed) = controller_call(ds4)

					#print(type(buttons_pressed))
					#print(buttons_pressed)

					################### switch modes #############################################
					#if 'circle' in ds4.presses:
					#    current_mode = Stable_4_legs(current_leg_locations)
					#elif_________:
					#   current_mode=_______________
					#___________
					#___________
					#___________
					################### switch modes #############################################



					shut_down = current_mode.plan_movement(ds4)
					if shut_down==True:
						sleep(current_mode.substep_delay)
						continue

					#need to change to is_finished_step
					if current_mode.is_finished_step == True:
						current_mode.calculate_points()

					current_mode.next_substep()
					for leg_num in range(4):
						execute_movement(i, current_mode.angles_servo[i][0])
						# current_leg_locations[leg_num] = points[leg_num][current_substep_num]
					current_mode.update_substep()

					#current_substep_num += 1
					#if (current_substep_num >= last_substep_num):
					#	is_finished_step = True
					# current_substep_num -= 1 ## this is here in case plan failed and is_changed == false

					time.sleep(current_mode.substep_delay)


				######### this is out of the while(1). Shutdown everything ########################

	except IOError:
		## set the robot to the starting zero position
		current_leg_locations = zero_position()
		#current_mode = Stable_4_legs(current_leg_locations)
		# No DS4 controller found, wait for a bit and try again
		print('Waiting for a DS4 controller connection')
		sleep(0.5)  # temp
		print("Euler angle: {}".format(sensor.euler))









"""
######### *temp* for stand mode ###### changed in shai
sensor_act = 1 # the value under which the robot is considered flat
def stand_pre_execution(sensor, sensor_offset):
	(euler1, euler2, euler3) = sensor.euler
	print((euler1, euler2, euler3))
	offsetsX= [0,0,0,0]
	offsetsY = [0,0,0,0]
	offsetsZ = [0,0,0,0]

	if (abs(euler2) > sensor_act):
		offsetsY[0] -= np.sign(euler2) * (1 + int(math.fabs(euler2)/5))
		offsetsY[1] -= np.sign(euler2) * (1 + int(math.fabs(euler2)/5))
		offsetsY[2] += np.sign(euler2) * (1 + int(math.fabs(euler2)/5))
		offsetsY[3] += np.sign(euler2) * (1 + int(math.fabs(euler2)/5))

		#offsetsZ[0] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))
		#offsetsZ[1] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))
		#offsetsZ[2] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))
		#offsetsZ[3] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))

	if (abs(euler3) > sensor_act):
		offsetsY[0] += np.sign(euler3) * (1 + int(math.fabs(euler3)/5))
		offsetsY[1] -= np.sign(euler3) * (1 + int(math.fabs(euler3)/5))
		offsetsY[2] -= np.sign(euler3) * (1 + int(math.fabs(euler3)/5))
		offsetsY[3] += np.sign(euler3) * (1 + int(math.fabs(euler3)/5))

		#offsetsX[0] += np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))
		#offsetsX[1] -= np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))
		#offsetsX[2] -= np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))
		#offsetsX[3] += np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))

	for j in range(4):
		lst = list(sensor_offset[j])
		lst[0] += offsetsX[j]
		lst[1] += offsetsY[j]
		lst[2] += offsetsZ[j]
		sensor_offset[j] = tuple(lst)
	print(sensor_offset)
	return sensor_offset
"""


"""
###########################################################
######### *temp* for 2-leg mode ###### changed in shai
sensor_act = 1 # the value under which the robot is considered flat
ratio = 1/3 # the ration between x and z, (z/x)
air_multiplier = 3
def two_leg_pre_execution(sensor, sensor_offset, two_leg_lean):
	(euler1, euler2, euler3) = sensor.euler
	print((euler1, euler2, euler3))
	offsetsX= [0,0,0,0]
	offsetsY = [0,0,0,0]
	offsetsZ = [0,0,0,0]

	if (two_leg_lean):
		if (abs(euler2) > sensor_act and abs(euler3) > sensor_act):
			offsetsZ[0] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5)) * ratio
			offsetsZ[2] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5)) * ratio

			offsetsX[0] += np.sign(euler3) * (1 + int(math.fabs(euler2) / 5))
			offsetsX[2] -= np.sign(euler3) * (1 + int(math.fabs(euler2) / 5))

			offsetsZ[1] -= np.sign(euler2) * (1 + int(math.fabs(euler2) / 5)) * air_multiplier * ratio
			offsetsZ[3] -= np.sign(euler2) * (1 + int(math.fabs(euler2) / 5)) * air_multiplier * ratio

			offsetsX[1] += np.sign(euler3) * (1 + int(math.fabs(euler2) / 5)) * air_multiplier
			offsetsX[3] -= np.sign(euler3) * (1 + int(math.fabs(euler2) / 5)) * air_multiplier
	else:
		if (abs(euler2) > sensor_act and abs(euler3) > sensor_act):
			offsetsZ[1] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5)) * ratio
			offsetsZ[3] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5)) * ratio

			offsetsX[1] -= np.sign(euler3) * (1 + int(math.fabs(euler2) / 5))
			offsetsX[3] += np.sign(euler3) * (1 + int(math.fabs(euler2) / 5))

			offsetsZ[0] -= np.sign(euler2) * (1 + int(math.fabs(euler2) / 5)) * air_multiplier * ratio
			offsetsZ[2] -= np.sign(euler2) * (1 + int(math.fabs(euler2) / 5)) * air_multiplier * ratio

			offsetsX[0] -= np.sign(euler3) * (1 + int(math.fabs(euler2) / 5)) * air_multiplier
			offsetsX[2] += np.sign(euler3) * (1 + int(math.fabs(euler2) / 5)) * air_multiplier


	for j in range(4):
		lst = list(sensor_offset[j])
		lst[0] += offsetsX[j]
		lst[1] += offsetsY[j]
		lst[2] += offsetsZ[j]
		sensor_offset[j] = tuple(lst)
	print(sensor_offset)
	return sensor_offset
"""


"""
## main old version


points = [[], [], [], []]
is_finished_step = True
all_angles = [[], [], [], []]
current_substep_num = 0
last_substep_num = 0
####current_substep_num = [0, 0, 0, 0]
####last_substep_num = [0, 0, 0, 0]
is_stand = True
is_2_leg = False
two_leg_lean = True ## True means legs 0 and 2 are touching the ground, False means legs 1 and 3 are touching the ground
two_leg_path = [[], [], [], []]
two_leg_offset = 40
two_leg_substeps = 80
two_leg_counter = 0


current_leg_locations = zero_position()
sleep(1)  # gives enough time to get to zero position

###
current_mode = Stable_4_legs(current_leg_locations)

## here starts communication with ps4 controller
while True:
	led.value = True
	time.sleep(0.5)
	led.value = False
	try:
		with ControllerResource(ControllerRequirement(require_class=DualShock4)) as ds4:
			while ds4.connected:
				#turn led on
				led.value = True
				print('Controller Connected')
				sensor_offset = [(0,0,0), (0,0,0), (0,0,0), (0,0,0)]
				# main loop
				while (ds4.connected):
					(left_cx, left_cy, right_cx, right_cy, buttons_pressed) = controller_call(ds4)
					print(type(buttons_pressed))
					print(buttons_pressed)


					################### TWO LEG SWITCH #########################
					if 'circle' in ds4.presses:
						print("hello yall")
						is_stand = False
						is_2_leg = True
						sensor_offset = [(0,0,0), (0,0,0), (0,0,0), (0,0,0)]
						two_leg_path[0] = calculate_points(current_leg_locations[0], current_leg_locations[0], 0, two_leg_substeps)
						two_leg_path[1] = calculate_points(current_leg_locations[1], current_leg_locations[1], 0, two_leg_substeps)
						two_leg_path[2] = calculate_points(current_leg_locations[2], current_leg_locations[2], 0, two_leg_substeps)
						two_leg_path[3] = calculate_points(current_leg_locations[3], current_leg_locations[3], 0, two_leg_substeps)
						if (two_leg_lean):
							(x,y,z) = current_leg_locations[1]
							two_leg_path[1] = calculate_points((x,y,z),(x,y+two_leg_offset,z),0,two_leg_substeps)
							(x, y, z) = current_leg_locations[3]
							two_leg_path[3] = calculate_points((x, y, z), (x, y + two_leg_offset, z), 0, two_leg_substeps)
						else:
							(x, y, z) = current_leg_locations[0]
							two_leg_path[0] = calculate_points((x, y, z), (x, y + two_leg_offset, z), 0, two_leg_substeps)
							(x, y, z) = current_leg_locations[2]
							two_leg_path[2] = calculate_points((x, y, z), (x, y + two_leg_offset, z), 0, two_leg_substeps)

					################### END OF TWO LEG SWITCH #########################



					###### STANDING MODE ########
					if is_stand:
						default_x = Settings.default_x
						default_y = Settings.default_y
						default_z = Settings.default_z
						sensor_offset = stand_pre_execution(sensor, sensor_offset)
						for i in range(4):
							(offset1, offset2, offset3) = Settings.legs_offset[i]
							if i>2: 
								offset3= offset3-10
							(sensor_offset1, sensor_offset2, sensor_offset3) = sensor_offset[i]
							(theta1, theta2, theta3) = legIK(default_x + offset1 + sensor_offset1, default_y + offset2 + sensor_offset2, default_z + offset3 + sensor_offset3)
							current_leg_locations[i] = (default_x + sensor_offset1, default_y + sensor_offset2, default_z + sensor_offset3)
							angles_servo = servo_angles([(theta1, theta2, theta3)], i)
							execute_movement(i, angles_servo[0])
							#execute_movement(i, current_mode.angles_servo[i][0])
						sleep(Settings.max_delay)
						continue
					###### END OF STANDING MODE ########

					###### 2 LEG MODE ########
					if is_2_leg:
						sensor_offset = two_leg_pre_execution(sensor, sensor_offset, two_leg_lean)
						for i in range(4):
							(offset1, offset2, offset3) = Settings.legs_offset[i]
							(x,y,z) = two_leg_path[i][two_leg_counter]
							if i > 2:
								offset3 = offset3 - 10
							(sensor_offset1, sensor_offset2, sensor_offset3) = sensor_offset[i]
							(theta1, theta2, theta3) = legIK(x + int(offset1) + sensor_offset1, y + int(offset2) + sensor_offset2, z + int(offset3) + sensor_offset3)
							angles_servo = servo_angles([(theta1, theta2, theta3)], i)
							execute_movement(i, angles_servo[0])
						sleep(Settings.max_delay)
						if (two_leg_counter < two_leg_substeps):
								two_leg_counter += 1
						continue
					##### END OF 2 LEG MODE ########










					(end_points, num_of_substeps, heights, substep_delay, is_changed, shut_down) = plan_movement(current_leg_locations, is_finished_step, ds4, is_stand)
					### for now just tests pausing movement
					if (shut_down == True):
						sleep(substep_delay)
						continue

					#need to change to is_finished_step
					if (is_changed == True):
						angles_rad = [[], [], [], []]
						is_finished_step = False
						current_substep_num = 0
						last_substep_num = num_of_substeps
						# calculate_movement(start_p, end_p, height, num_of_substeps):
						# all_angles should be in servo-angles
						for leg_num in range(4):
							print((leg_num, current_leg_locations[leg_num], end_points[leg_num], num_of_substeps, substep_delay, is_changed, shut_down))
							print(heights[leg_num])
							angles_rad[leg_num] = calculate_movement(current_leg_locations[leg_num], end_points[leg_num], heights[leg_num], num_of_substeps) #### make sure that calculate movement is designed for both left and right
							all_angles[leg_num] = servo_angles(angles_rad[leg_num], leg_num)
							points[leg_num] = calculate_points(current_leg_locations[leg_num], end_points[leg_num], heights[leg_num], num_of_substeps) ###same as calculate movement

					for leg_num in range(4):
						execute_movement(leg_num, all_angles[leg_num][current_substep_num])
					#### current_mode.update_substep()
						current_leg_locations[leg_num] = points[leg_num][current_substep_num]
					current_substep_num += 1


					if (current_substep_num >= last_substep_num):
						is_finished_step = True
					# current_substep_num -= 1 ## this is here in case plan failed and is_changed == false

					time.sleep(substep_delay)


				######### this is out of the while(1). Shutdown everything ########################

	except IOError:
		## set the robot to the starting zero position
		current_leg_locations = zero_position()
		# No DS4 controller found, wait for a bit and try again
		print('Waiting for a DS4 controller connection')
		sleep(0.5)  # temp
		print("Euler angle: {}".format(sensor.euler))
"""







