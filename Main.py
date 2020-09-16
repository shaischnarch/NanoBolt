from Calculations.All_calculations import *
from Excution.All_executions import *
from Plan.All_Plans import *
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


######### *temp* for stand mode ###### changed in shai
sensor_act = 1 # the value under which the robot is considered flat
def stand_pre_execution(sensor, sensor_offset):

	(euler1, euler2, euler3) = sensor.euler
	print((euler1, euler2, euler3))
	offsets = [0,0,0,0]
	if math.fabs(euler2)>sensor_act or math.fabs(euler3)>sensor_act:
		lowest = 0
		diff_width = -round(math.tan(
			math.pi * math.fabs(euler3) / 180) * Settings.robot_width)  ## math.tan gets rad, so we invert euler angle
		diff_length = -round(math.tan(
			math.pi * math.fabs(euler2) / 180) * Settings.robot_length)  ## math.tan gets rad, so we invert euler angle
		lowest_width_neighber = diff_width + lowest
		lowest_length_neighber = diff_length + lowest
		highest = lowest + diff_width + diff_length

		if (euler2 > 0 and euler3 > 0):  ## leg 1 is the highest, leg 3 is the lowest
			leg0 = lowest_length_neighber
			leg1 = highest
			leg2 = lowest_width_neighber
			leg3 = lowest

		elif (euler2 > 0 and euler3 < 0):  ## leg 0 is the highest, leg 2 is the lowest
			leg0 = highest
			leg1 = lowest_length_neighber
			leg2 = lowest
			leg3 = lowest_width_neighber

		elif (euler2 < 0 and euler3 > 0):  ## leg 2 is the highest, leg 0 is the lowest
			leg0 = lowest
			leg1 = lowest_width_neighber
			leg2 = highest
			leg3 = lowest_length_neighber

		else:  ## leg 3 is the highest, leg 1 is the lowest
			leg0 = lowest_width_neighber
			leg1 = lowest
			leg2 = lowest_length_neighber
			leg3 = highest

		offsets[0] = round((leg0 - leg2) / 2)
		offsets[1] = round((leg1 - leg3) / 2)
		offsets[2] = round((leg2 - leg0) / 2)
		offsets[3] = round((leg3 - leg1) / 2)

	for j in range(4):
		lst = list(sensor_offset[j])
		lst[1] = offsets[j]
		sensor_offset[j] = tuple(lst)
	print(sensor_offset)
	return sensor_offset

###########################################################


# Led Pin Definitions
led = digitalio.DigitalInOut(board.D18)
led.direction = digitalio.Direction.OUTPUT

# IMU SETUP AND Definitions
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

## initialization
servo_setup()  # starts i2c communication with servos




points = [[], [], [], []]
is_finished_step = True
all_angles = [[], [], [], []]
current_substep_num = 0
last_substep_num = 0
####current_substep_num = [0, 0, 0, 0]
####last_substep_num = [0, 0, 0, 0]
is_stand = True

current_leg_locations = zero_position()
sleep(1)  # gives enough time to get to zero position


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

					###### STANDING MODE ########
					if is_stand:
						default_x = Settings.default_x
						default_y = Settings.default_y
						default_z = Settings.default_z
						sensor_offset = stand_pre_execution(sensor, sensor_offset)
						for i in range(4):
							(offset1, offset2, offset3) = Settings.legs_offset[i]
							(sensor_offset1, sensor_offset2, sensor_offset3) = sensor_offset[i]
							(theta1, theta2, theta3) = legIK(default_x + offset1 + sensor_offset1, default_y + offset2 + sensor_offset2, default_z + offset3 + sensor_offset3)
							angles_servo = servo_angles([(theta1, theta2, theta3)], i)
							execute_movement(i, angles_servo[0])
						sleep(Settings.max_delay)
						continue
					###### END OF STANDING MODE ########





					(end_points, num_of_substeps, heights, substep_delay, is_changed, shut_down) = plan_movement(current_leg_locations, is_finished_step, ds4, is_stand)
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
							print((leg_num, current_leg_locations[leg_num], end_points[leg_num], num_of_substeps, substep_delay, is_changed, shut_down))
							print(heights[leg_num])
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
		current_leg_locations = zero_position()
		# No DS4 controller found, wait for a bit and try again
		print('Waiting for a DS4 controller connection')
		sleep(0.5)  # temp
		print("Euler angle: {}".format(sensor.euler))


