from Helper_directory import Settings
from Helper_directory.Main_helper import move_to_stand, move_to_position, read_sensor
from Calculations.All_calculations import calculate_points, servo_angles, legIK
import numpy as np
import math
from Modes_directory.Mode_Class import Mode


## Walking mode where both legs that are diagonal to each other do the same operation at the same time
# FOR NOW this class is a test for the feasibility of the mode
# TODO: After tests, if mode is feasible, implement the proper mode
class Walking_2_legs(Mode):

    def __init__(self, current_legs_location):
        Mode.__init__(self, current_legs_location)

        # offset from stand position
        self.semi_ideal_current_pos, self.current_legs_location = move_to_position(Settings.first_diag_forward_default[:], self.current_legs_location)

        self.pause_movement = False
        self.is_finished_step = True
        self.forward = True  # when true walk forward, false walk backwards

        # 0 means that the current step being executed is from second diag to first, 1 means the opposite
        # when diag is 0, legs 0,2 move forward in the air, and 1,3 backwards on the ground
        self.diag = 0

        self.height_jumps = 1 # change for testing
        self.max_leg_height = 30  # the max height offset from the controller (in both up and down directions)
        self.height = 15 ## staring height # change for height limition
        # the leg position controlled by the left stick

        # change for testing
        # define max and min number of substep:
        self.num_of_substeps_jumps = 2
        self.max_num_of_substep = 36
        self.min_num_of_substep = 4

        # change for testing
        # define max and min substep delay:
        self.substep_delay_jumps = 0.005
        self.max_substep_delay = Settings.max_delay + self.substep_delay_jumps*8
        self.min_substep_delay = Settings.min_delay

        ## sensors variables
        self.sensor_active = 0
        self.num_of_substeps_sensor_checker = 2
        self.sensor_angle_unit = 5  # it means that every time,we add another correction units for every additional 5 degrees diffrence from wanted angle
        self.sensor_z_axis_offset_units = 1
        self.sensor_x_axis_offset_units = 1
        self.sensor_y_axis_offset_units = 1

        # define the wanted sensors angles and what is the maximum distance allowed from those angles
        self.wanted_front_angle = 0
        self.front_angle_allowed_diff = 1
        self.wanted_side_angle = 0
        self.side_angle_allowed_diff = 3

    #OVERRIDE
    def plan_movement(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):

        if self.diag == 0:
            self.diag = 1
            self.heights = [0, self.height, 0, self.height]
            if self.forward:
                self.end_points = Settings.second_diag_forward_default[:]
            else:
                self.end_points = Settings.second_diag_backward_default[:]

        else:
            self.diag = 0
            self.heights = [self.height, 0, self.height, 0]  # change
            if self.forward:
                self.end_points = Settings.first_diag_forward_default[:]
            else:
                self.end_points = Settings.first_diag_backward_default[:]





    """
        if self.diag == 0:
            self.diag = 1
            self.end_points = Settings.second_diag_default[:]
            if self.forward:
                self.heights = [0, self.height, 0, self.height]
                self.sensor_offset[1] = (0, 0, 0)  # change
                self.sensor_offset[3] = (0, 0, 0)  # change
                self.controller_offset[1] = (0, 0, 0)  # change
                self.controller_offset[3] = (0, 0, 0)  # change
            else:
                self.heights = [self.height, 0, self.height, 0]     ##### TEST, MIGHT DELETE
                self.sensor_offset[0] = (0, 0, 0)  # change
                self.sensor_offset[2] = (0, 0, 0)  # change
                self.controller_offset[0] = (0, 0, 0)  # change
                self.controller_offset[2] = (0, 0, 0)  # change


        else:
            self.diag = 0
            self.end_points = Settings.first_diag_default[:]
            if self.forward:
                self.heights = [self.height, 0, self.height, 0]  # change
                self.sensor_offset[0] = (0, 0, 0)  # change
                self.sensor_offset[2] = (0, 0, 0)  # change
                self.controller_offset[0] = (0, 0, 0)  # change
                self.controller_offset[2] = (0, 0, 0)  # change
            else:
                self.heights = [0, self.height, 0, self.height]     ##### TEST, MIGHT DELETE
                self.sensor_offset[1] = (0, 0, 0)  # change
                self.sensor_offset[3] = (0, 0, 0)  # change
                self.controller_offset[1] = (0, 0, 0)  # change
                self.controller_offset[3] = (0, 0, 0)  # change
    """


    # @OVERRIDE - add the ability to calculate the removal of offsets for the correct legs
    # Main function for calculating all the substeps locations along the a step.
    # Therefore it is called only after plan_movement and it finishes setting up the new step.
    # @Receives: nothing
    # @Resets: is_finished_step = false, current_substep = 1
    # @Returns nothing
    def calculate_points(self):
        for leg_num in range(4):
            # For this mode, we want to remove the phisical offsets after each step, for this reason we calculate points from current legs location and not semi ideal
            self.points[leg_num] = calculate_points(self.current_legs_location[leg_num], self.end_points[leg_num], self.heights[leg_num],
                                                    self.num_of_substeps)
            self.sensor_offset[leg_num] = (0, 0, 0)  # change
            self.controller_offset[leg_num] = (0, 0, 0)  # change
        # self.current_substep = 0
        self.current_substep = 1  # EXPERIMENTAL, might need to be lowered back down to 0
        self.is_finished_step = False
        """
        if (self.diag == 1 and self.forward) or (self.diag == 0 and not self.forward):
            self.points[0] = calculate_points(self.semi_ideal_current_pos[0], self.end_points[0], self.heights[0], self.num_of_substeps)
            self.points[2] = calculate_points(self.semi_ideal_current_pos[2], self.end_points[2], self.heights[2], self.num_of_substeps)
            self.points[1] = calculate_points(self.current_legs_location[1], self.end_points[1], self.heights[1], self.num_of_substeps)
            self.points[3] = calculate_points(self.current_legs_location[3], self.end_points[3], self.heights[3], self.num_of_substeps)
        else:
            self.points[1] = calculate_points(self.semi_ideal_current_pos[1], self.end_points[1], self.heights[1], self.num_of_substeps)
            self.points[3] = calculate_points(self.semi_ideal_current_pos[3], self.end_points[3], self.heights[3], self.num_of_substeps)
            self.points[0] = calculate_points(self.current_legs_location[0], self.end_points[0], self.heights[0], self.num_of_substeps)
            self.points[2] = calculate_points(self.current_legs_location[2], self.end_points[2], self.heights[2], self.num_of_substeps)
        # self.current_substep = 0
        self.current_substep = 1  # EXPERIMENTAL, might need to be lowered back down to 0
        self.is_finished_step = False
        """





    # for now, use the controller for 3 changes: substep delay, max height, and num of substpe changes for testing.
    # todo: implement proper usage of contreller later
    def controller_input(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):

    # change (all of that)
        # Activate / DeActivate Sensor
        if 'dup' in buttons_pressed:
            self.sensor_active = not self.sensor_active
            print("sensor_active: " + str(self.sensor_active))
            # Note:  The current sensor offset is still in the system, we are not removing it to not cause a sudden leg position jump
            #        It will automatically be removed after one or two steps

        # Change walking direction
        if 'ddown' in buttons_pressed:
            self.forward = not self.forward
            self.wanted_front_angle *= -1
            print("Walking Forward: " + str(self.forward))

        # Pause / UnPause movement
        if 'r1' in buttons_pressed:
            self.pause_movement = not self.pause_movement
            print("paused: " + str(self.pause_movement))
        # change num of substep
        if 'dright' in buttons_pressed:
            if self.num_of_substeps<self.max_num_of_substep:
                self.num_of_substeps += self.num_of_substeps_jumps
                print("Updated current num of substeps to: " + str(self.num_of_substeps))
            else:
                print("Num of substep is already in it's max value: "+ str(self.max_num_of_substep) )
        elif 'dleft' in buttons_pressed:
            if self.num_of_substeps>self.min_num_of_substep:
                self.num_of_substeps -= self.num_of_substeps_jumps
                print("Updated current num of substeps to: " + str(self.num_of_substeps))
            else:
                print("Num of substep is already in it's min value: " + str(self.min_num_of_substep))

        #change height, war
        if left_cy > 0 and self.height < self.max_leg_height:
            self.height += self.height_jumps
            print("Updated step height to: " + str(self.height))
        elif left_cy > 0:
            print("Height is already in it's max value: " + str(self.max_leg_height))

        if left_cy < 0 and self.height > -self.max_leg_height:
            self.height -= self.height_jumps
            print("Updated step height to: " + str(self.height))
        elif left_cy < 0:
            print("Height is already in it's min value: " + str(-self.max_leg_height))

    # change substep delay , defult is max= 0.05, min according to settings is 0.0075
        if right_cy > 0 and self.substep_delay < self.max_substep_delay:
            self.substep_delay += self.substep_delay_jumps
            print("Updated substep delay to: " + str(self.substep_delay))
        elif right_cy > 0:
            print("Substep delay is already in it's max value: " + str(self.max_substep_delay))

        if right_cy < 0 and self.substep_delay > self.min_substep_delay:
            self.substep_delay -= self.substep_delay_jumps
            print("Updated substep delay to: " + str(self.substep_delay))
        elif right_cy < 0:
            print("Substep delay is already in it's min value: " + str(self.min_substep_delay))


    # This method is called upon before executing the next substep.
    # When this method is called, the location of the next semi-ideal substep is already available in self.points[leg_num][self.current_substep]
    # Therefore this method uses the sensor to make realtime minor changes to the next substep - transfer to physical/dynamic realm
    # It than calculates the actual values for the servos to execute the next substep
    # @updates: current_legs_location with the real time location, and semi_ideal_current_pos with the semi ideal position
    # todo: might be a cause of error, debug later
    def prep_substep(self, sensor):
        if self.current_substep % self.num_of_substeps_sensor_checker == 0 and self.sensor_active == 1:
            self.__Walking_2_legs_sensor_helper(sensor)
        self.sum_offsets()
        for leg_num in range(4):
            (point_x, point_y, point_z) = self.points[leg_num][self.current_substep]  # The calculated target location
            (offset_x, offset_y, offset_z) = self.offsets[leg_num]
            x = point_x + offset_x
            y = point_y + offset_y
            z = point_z + offset_z
            #print("leg {} final point value: {}".format(leg_num, (x,y,z)))
            try:
                (theta1, theta2, theta3) = legIK(x, y, z)
                self.angles_servo[leg_num] = servo_angles([(theta1, theta2, theta3)], leg_num)
                self.current_legs_location[leg_num] = (x, y, z)  # real location
                self.semi_ideal_current_pos[leg_num] = (point_x, point_y, point_z)  # semi-ideal location
            except:
                print('ERROR: Tried to move to impossible position')
                self.stop_movement = True



    def __Walking_2_legs_sensor_helper(self, sensor):
        (euler1, euler2, euler3) = read_sensor(sensor)
        print((euler1, euler2, euler3))
        offsetsX = [0, 0, 0, 0]
        offsetsY = [0, 0, 0, 0]
        offsetsZ = [0, 0, 0, 0]

        synced_front_angle = euler2 - self.wanted_front_angle
        synced_side_angle = euler3 - self.wanted_side_angle
        if self.diag == 0:  ## legs 0 and 2 are in the air, while legs 1 and 3 on the ground
            if (abs(synced_front_angle) > self.front_angle_allowed_diff):
                ##offsetsY[0] -= np.sign(synced_front_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                offsetsY[1] -= np.sign(synced_front_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                ##offsetsY[2] += np.sign(synced_front_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                offsetsY[3] += np.sign(synced_front_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))

                ##offsetsZ[0] += np.sign(synced_front_angle) * (self.sensor_z_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                offsetsZ[1] += np.sign(synced_front_angle) * (self.sensor_z_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                ##offsetsZ[2] += np.sign(synced_front_angle) * (self.sensor_z_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                offsetsZ[3] += np.sign(synced_front_angle) * (self.sensor_z_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))

            if (abs(synced_side_angle) > self.side_angle_allowed_diff):
                ##offsetsY[0] += np.sign(synced_side_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                offsetsY[1] -= np.sign(synced_side_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                ##offsetsY[2] -= np.sign(synced_side_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                offsetsY[3] += np.sign(synced_side_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))

                ##offsetsX[0] += np.sign(synced_side_angle) * (self.sensor_x_axis_offset_units+int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                offsetsX[1] -= np.sign(synced_side_angle) * (self.sensor_x_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                ##offsetsX[2] -= np.sign(synced_side_angle) * (self.sensor_x_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                offsetsX[3] += np.sign(synced_side_angle) * (self.sensor_x_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))

        elif self.diag == 1: ## legs 1 and 3 are in the air, legs 0 and 2 on the ground.
            if (abs(synced_front_angle) > self.front_angle_allowed_diff):
                offsetsY[0] -= np.sign(synced_front_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                ## offsetsY[1] -= np.sign(synced_front_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                offsetsY[2] += np.sign(synced_front_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                ## offsetsY[3] += np.sign(synced_front_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))

                offsetsZ[0] += np.sign(synced_front_angle) * (self.sensor_z_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                ##offsetsZ[1] += np.sign(synced_front_angle) * (self.sensor_z_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                offsetsZ[2] += np.sign(synced_front_angle) * (self.sensor_z_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))
                ##offsetsZ[3] += np.sign(synced_front_angle) * (self.sensor_z_axis_offset_units + int(math.fabs(synced_front_angle) / self.sensor_angle_unit))

            if (abs(synced_side_angle) > self.side_angle_allowed_diff):
                offsetsY[0] += np.sign(synced_side_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                ## offsetsY[1] -= np.sign(synced_side_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                offsetsY[2] -= np.sign(synced_side_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                ## offsetsY[3] += np.sign(synced_side_angle) * (self.sensor_y_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))

                offsetsX[0] += np.sign(synced_side_angle) * (self.sensor_x_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                ##offsetsX[1] -= np.sign(synced_side_angle) * (self.sensor_x_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                offsetsX[2] -= np.sign(synced_side_angle) * (self.sensor_x_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))
                ##offsetsX[3] += np.sign(synced_side_angle) * (self.sensor_x_axis_offset_units + int(math.fabs(synced_side_angle) / self.sensor_angle_unit))

        for j in range(4):
            lst = list(self.sensor_offset[j])
            lst[0] += offsetsX[j]
            lst[1] += offsetsY[j]
            lst[2] += offsetsZ[j]
            self.sensor_offset[j] = tuple(lst)
