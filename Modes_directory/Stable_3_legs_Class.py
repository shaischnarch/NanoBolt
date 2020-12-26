from Helper_directory import Settings
from Helper_directory.Main_helper import move_to_stand, move_to_position
from Calculations.All_calculations import calculate_points, servo_angles, legIK
import numpy as np
import math
from Modes_directory.Mode_Class import Mode

## Mode inherited class - Stand on 3 legs
#  In this mode the robot uses the imu sensor to keep its self standing level while one leg is in the air.
#  This leg is always on the front, and can be switched between left and right.
#  To go into this mode: press TRIANGLE on the controller when in stand mode (Mode)
# todo: change control scheme
#
#  Controls: Left Stick Up - Move leg in the air up
#            Left Stick Down - Move leg in the air down
#            DPad Left - switch the leg in the air to the front left leg. If front left is already in the air, reset it to default position
#            DPad Right - switch the leg in the air to the front right leg. If front right is already in the air, reset it to default position
#            Right Stick - move the leg in the air in the corresponding direction
#            L1 - Feast bump with the leg that's in the air
class Stable_3_legs(Mode):

    def __init__(self, current_legs_location):
        Mode.__init__(self, current_legs_location)
        self.pause_movement = False
        self.is_finished_step = True
        self.leg_in_air = 1  # 1 is front right leg, 0 is front left leg
        self.leg_height = 0  # an addition to the height that is controlled by the controller, positive is leg lifted higher
        self.max_leg_height = 30  # the max height offset from the controller (in both up and down directions)
        # the leg position controlled by the left stick
        self.max_offset = 30
        self.leg_offset_x = 0  # positive is inside (take which leg is in the air into consideration)
        self.leg_offset_z = 0  # positive is forward
        self.controller_offset = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]  # leg movement as set by the controller

        # This variable is used to determine the action of the robot in this mode.
        # -1 and below - move robot to starting 3 legs position, starts at -1 and continues going down
        # 0 - robot is in stable_3_legs_mode, and the user can control its leg
        # 1 and above - robot is in fist bump mode, each step inside the fist bump mode is a new value
        self.action = -1
        
        self.fist_bump_delay = 8  # how long to delay retracting the arm, actual delay = self.fist_bump_delay * self.substep_delay
        # used for making the fist bump happen, its values are updated in plan_movement
        self.fist_bump = (0, 0, 0)

        # self.num_of_substeps = 1
        # self.sensor_act = 1


    def plan_movement(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):

        # move robot to stable 3_legs_mode:
        # first step: Raise robot and lean, this step is needed because the robot has a hard time getting up with 2 legs.
        if self.action == -1:
            self.num_of_substeps = 32
            self.action = -2
            #  set all legs to their final target position, but let all the legs help lift
            for i in range(4):
                if self.leg_in_air == 1:
                    (x, y, z) = Settings.stable_3_legs_right_default[i]
                    if i == 1 or i == 3:
                        y = -5  # raise the robot using all legs: these two legs drop for the final position, so here we raise them to help lift
                else:
                    (x, y, z) = Settings.stable_3_legs_left_default[i]
                    if i == 0 or i == 2:
                        y = -5  # raise the robot using all legs: these two legs drop for the final position, so here we raise them to help lift
                self.end_points[i] = (x, y, z)
            return


        # second step: lift leg in the air
        # uses the default_left/right_offsets, change this values to change the starting 3 legs position
        if self.action == -2:
            self.action = -3
            for i in range(4):
                if self.leg_in_air == 1:
                    (x, y, z) = Settings.stable_3_legs_right_default[i]
                else:
                    (x, y, z) = Settings.stable_3_legs_left_default[i]
                self.end_points[i] = (x, y, z)
            return


        # lastly, let the last step finish, than switch to action = 0
        if self.action == -3:
            self.action = 0
            return

        # if robot is in stable_3_legs_mode
        if self.action == 0:
            print("action0")
            return


        # fist bump step 1 - move give fist forward
        elif self.action == 1:
            fist_bump_len = 70  # how far to extend fist bump, in millimeters
            fist_bump_height = 15  # how far to raise the fist bump
            print("action1")
            self.action = 2
            self.fist_bump = (0, fist_bump_height, fist_bump_len)

        # fist bump step 2 - fist bump delay, no need to update step_offsets, just waits self.fist_bump_delay * self.substep_delay time
        elif self.action == 2:
            print("action2")
            self.action = 3
            self.num_of_substeps = self.fist_bump_delay

        # final fist bump step - move back to starting position
        elif self.action == 3:
            print("action3")
            self.action = 4
            self.num_of_substeps = 12
            self.fist_bump = (0,0,0)

        elif self.action == 4:
            self.action = 0
            return

        # update end points location with fist bump additions
        if self.leg_in_air == 1:
            self.end_points = Settings.stable_3_legs_right_default
        else:
            self.end_points = Settings.stable_3_legs_left_default

        (fist_x, fist_y, fist_z) = self.fist_bump
        (x, y, z) = self.end_points[self.leg_in_air]
        self.end_points[self.leg_in_air] = (x + fist_x, y + fist_y, z + fist_z)



    # Overwrite regular calculate_points when in leg in air mode, when in fist bump, use regular.
    # When in air mode: make points stay the same value. This is done because the movement of the leg is done using offsets
    def calculate_points(self):
        if self.action == 0:
            for i in range(4):
                self.points[i][0] = self.semi_ideal_current_pos
            self.num_of_substeps = 0
            self.current_substep = 0
        else:
            Mode.calculate_points(self)


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
