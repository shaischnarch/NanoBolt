from Helper_directory import Settings
from Helper_directory.Main_helper import move_to_stand, move_to_position
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
        self.semi_ideal_current_pos, self.current_legs_location = move_to_position(Settings.first_diag_default[:], self.current_legs_location)

        self.pause_movement = False
        self.is_finished_step = True

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
        self.max_substep_delay = Settings.max_delay+ self.substep_delay_jumps*8
        self.min_substep_delay = Settings.min_delay

    def plan_movement(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):

        if self.diag == 0:
            self.diag = 1
            self.end_points = Settings.second_diag_default[:]
            self.heights = [0,self.height,0,self.height] # change

        else:
            self.diag = 0
            self.end_points = Settings.first_diag_default[:]
            self.heights = [self.height, 0, self.height, 0] # change




    # for now, use the controller for 3 changes: substep delay, max height, and num of substpe changes for testing.
    # todo: implement proper usage of contreller later
    def controller_input(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):

    # change (all of that)
        # change num of substep
        if 'dright' in buttons_pressed:
            if self.num_of_substeps<self.max_num_of_substep:
                self.num_of_substeps += self.num_of_substeps_jumps
            else:
                print("Num of substep is already in it's max value: "+ str(self.max_num_of_substep) )
        elif 'dleft' in buttons_pressed:
            if self.num_of_substeps>self.min_num_of_substep:
                self.num_of_substeps -= self.num_of_substeps_jumps
            else:
                print("Num of substep is already in it's min value: " + str(self.min_num_of_substep))

        #change height, war
        if left_cy > 0 and self.height < self.max_leg_height:
            self.height += self.height_jumps
        else:
            print("Height is already in it's max value: " + str(self.max_leg_height))

        if left_cy < 0 and self.height > -self.max_leg_height:
            self.height -= self.height_jumps
        else:
            print("Height is already in it's min value: " + str(-self.max_leg_height))

    # change substep delay , defult is max= 0.05, min according to settings is 0.0075
        if right_cy > 0 and self.substep_delay < self.max_substep_delay:
            self.substep_delay += self.substep_delay_jumps
        else:
            print("Substep delay is already in it's max value: " + str(self.max_substep_delay))

        if right_cy < 0 and self.substep_delay > self.min_substep_delay:
            self.substep_delay -= self.substep_delay_jumps
        else:
            print("Substep delay is already in it's min value: " + str(self.min_substep_delay))


    # for now use the version form Mode. todo: write a version to take the sensor into consideration
    # def prep_substep(self, sensor):
    #     pass

