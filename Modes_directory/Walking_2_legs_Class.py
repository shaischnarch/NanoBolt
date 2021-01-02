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


        self.max_leg_height = 30  # the max height offset from the controller (in both up and down directions)
        # the leg position controlled by the left stick



    def plan_movement(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):

        if self.diag == 0:
            self.diag = 1
            self.end_points = Settings.second_diag_default[:]
            self.heights = [0,self.max_leg_height,0,self.max_leg_height]

        else:
            self.diag = 0
            self.end_points = Settings.first_diag_default[:]
            self.heights = [self.max_leg_height, 0, self.max_leg_height, 0]




    # Receives input from the controller.
    # Changes leg position, leg height, and switch leg
    # Controls are writen in Class description
    def controller_input(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):
        if 'square' in buttons_pressed:  # 'square' is actually L1 on the ps4 controller, bad controller library
            self.action = 1  # go into fist bump mode
            self.is_finished_step = True  # start immediately the fist bump
            self.controller_offset = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
            # move robot back to default position
            if self.leg_in_air == 1:
                self.semi_ideal_current_pos, self.current_legs_location = move_to_position(Settings.stable_3_legs_right_default, self.current_legs_location)
            else:
                self.semi_ideal_current_pos, self.current_legs_location = move_to_position(Settings.stable_3_legs_left_default, self.current_legs_location)
            return

        # allow controller input only when not in fist bump mode
        if self.action == 0:
            (offset_x, offset_y, offset_z) = self.controller_offset[self.leg_in_air]
            if 'dright' in buttons_pressed:
                if self.leg_in_air == 1:
                    self.semi_ideal_current_pos, self.current_legs_location = move_to_position(Settings.stable_3_legs_right_default, self.current_legs_location)
                else:
                    self.semi_ideal_current_pos, self.current_legs_location = move_to_stand(self.current_legs_location)
                    self.action = -1  # Move robot back to starting 3 legs position
                offset_x = 0
                offset_y = 0
                offset_z = 0
                self.leg_in_air = 1

            elif 'dleft' in buttons_pressed:
                if self.leg_in_air == 0:
                    self.semi_ideal_current_pos, self.current_legs_location = move_to_position(Settings.stable_3_legs_left_default, self.current_legs_location)
                else:
                    self.semi_ideal_current_pos, self.current_legs_location = move_to_stand(self.current_legs_location)
                    self.action = -1  # Move robot back to starting 3 legs position
                offset_x = 0
                offset_y = 0
                offset_z = 0
                self.leg_in_air = 0

            if left_cy > 0 and offset_y < self.max_leg_height:
                offset_y += 1
            if left_cy < 0 and offset_y > -self.max_leg_height:
                offset_y -= 1

            # Control movement in z direction
            if right_cy > 0 and offset_z < self.max_offset:
                offset_z += 1
            if right_cy < 0 and offset_z > -self.max_offset:
                offset_z -= 1

            # Control movement in the x direction. Positive x is towards the inside of the robot, so we must take leg_in_air into consideration
            if self.leg_in_air == 0:
                if right_cx > 0 and offset_x < self.max_offset:
                    offset_x += 1
                if right_cx < 0 and offset_x > -self.max_offset:
                    offset_x -= 1
            else:
                if right_cx > 0 and offset_x > -self.max_offset:
                    offset_x -= 1
                if right_cx < 0 and offset_x < self.max_offset:
                    offset_x += 1

            self.controller_offset[self.leg_in_air] = (offset_x, offset_y, offset_z)  # leg movement as set by the controller

    # for now use the version form Mode. todo: write a version to take the sensor into consideration
    # def prep_substep(self, sensor):
    #     pass

    # This method preps the transition to lifting up one leg.
    # The way it does this is by creating the first step for all the legs to reach the desired location.
    # This methods takes into consideration the leg that needs to be lifted, and updates the endpoints accordingly
    # Note: this method sets is_finished_step = false in order for the robot to consider this setup a step
    # Currently, move legs as if robot is a square, might update according to results
    # def stand_to_3_legs(self):
    #     for i in range(4):
    #         (def_x, def_y, def_z) = Settings.default_with_offset[i]
    #         if self.leg_in_air == 1:
    #             (offset_x, offset_y, offset_z) = self.default_right_offset[i]
    #         else:
    #             (offset_x, offset_y, offset_z) = self.default_left_offset[i]
    #         self.end_points[i] = (def_x + offset_x, def_y + offset_y, def_z + offset_z)
    #
    #     self.is_finished_step = False
    #     Mode.calculate_points(self)
    #
