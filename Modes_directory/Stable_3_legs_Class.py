from Helper_directory import Settings
from Helper_directory.Main_helper import move_to_stand
from Calculations.All_calculations import calculate_points, servo_angles, legIK
import numpy as np
import math
from Modes_directory.Mode_Class import Mode

## Mode inherited class - Stand on 3 legs
#  In this mode the robot uses the imu sensor to keep its self standing level while one leg is in the air.
#  This leg is always on the front, and can be switched between left and right.
#  To go into this mode: press TRIANGLE on the controller when in stand mode (Mode)
#  Controls: DPad Up - Move leg in the air up
#            DPad Down - Move leg in the air down
#            DPad Left - switch the leg in the air to the front left leg. If front left is already in the air, reset it to default position
#            DPad Right - switch the leg in the air to the front right leg. If front right is already in the air, reset it to default position
#            Left stick - move the leg in the air in the corresponding direction
#            L1 - Feast bump with the leg that's in the air
class Stable_3_legs(Mode):

    def __init__(self, current_legs_location):
        Mode.__init__(self, current_legs_location)
        self.pause_movement = False
        self.leg_in_air = 1  # 1 is front right leg, 0 is front left leg
        self.leg_height = 0  # an addition to the height that is controlled by the controller, positive is leg lifted higher
        self.max_leg_height = 30  # the max height offset from the controller (in both up and down directions)
        # the leg position controlled by the left stick
        self.max_offset = 30
        self.leg_offset_x = 0  # positive is inside (take which leg is in the air into consideration)
        self.leg_offset_z = 0  # positive is forward


        self.default_right_offset = [(15,0,15), (0,30,0), (-15,0,15), (0,10,0)]  # The offset from default position to stand on 3 legs when right leg is in the air
        self.default_left_offset = [(0,30,0), (15,0,15), (0,10,0), (-15,0,15)]  # The offset from default position to stand on 3 legs when left leg is in the air
        self.stand_to_3_legs()  # setup transition step

        # This variable is used to determine the action of the robot in this mode.
        # 0 - robot is in stable_3_legs_mode, and the user can control its leg
        # 1- - robot is in fist bump mode, each step inside the fist bump mode is a new value
        # final - the final step in the fist bump mode, after this the robot returns to action = 0
        self.action = 0
        self.fist_bump_len = 35  # how far to extend fist bump, in millimeters
        self.fist_bump_delay = 5  # how long to delay retracting the arm, actual delay = self.fist_bump_delay * self.substep_delay
        # used for making the fist bump happen, its values are updated in plan_movement
        self.fist_bump_offsets = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]

        # self.num_of_substeps = 1
        # self.sensor_act = 1


    def plan_movement(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):
        # if robot is in stable_3_legs_mode
        if self.action == 0:
            self.num_of_substeps = 1
            self.end_points = self.current_legs_location
            return

        # start of fist bump mode - move leg into default position
        if self.action == 1:
            self.num_of_substeps = 32
            self.action = 2
            self.fist_bump_offsets = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]

        # fist bump step 2 - move give fist forward
        elif self.action == 2:
            self.action = 3
            if self.leg_in_air == 1:
                self.fist_bump_offsets[1] = (0, 0, self.fist_bump_len)
            else:
                self.fist_bump_offsets[0] = (0, 0, self.fist_bump_len)

        # fist bump step 3 - fist bump delay, no need to update step_offsets, just waits self.fist_bump_delay * self.substep_delay time
        elif self.action == 3:
            self.action = 4
            self.num_of_substeps = self.fist_bump_delay

        # final fist bump step - move back to starting position
        elif self.action == 4:
            self.action = 0
            self.num_of_substeps = 32
            self.fist_bump_offsets = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]

        for i in range(4):
            (def_x, def_y, def_z) = Settings.default_with_offset[i]
            (fist_x, fist_y, fist_z) = self.fist_bump_offsets[i]
            if self.leg_in_air == 1:
                (offset_x, offset_y, offset_z) = self.default_right_offset[i]
            else:
                (offset_x, offset_y, offset_z) = self.default_left_offset[i]
            self.end_points[i] = (def_x + offset_x + fist_x, def_y + offset_y + fist_y, def_z + offset_z + fist_z)




    # Receives input from the controller.
    # Changes leg position, leg height, and switch leg
    # Controls are writen in Class description
    def controller_input(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):
        if 'square' in buttons_pressed:  # 'square' is actually L1 on the ps4 controller, bad controller library
            self.action = 1  # go into fist bump mode
            self.is_finished_step = True  # start immediately the fist bump
            return

        # allow controller input only when not in fist bump mode
        if self.action == 0:
            if 'dright' in buttons_pressed:
                move_to_stand(current_legs_location)
                self.leg_height = 0
                self.leg_offset_x = 0
                self.leg_offset_z = 0
                self.leg_in_air = 1
            elif 'dleft' in buttons_pressed:
                move_to_stand(current_legs_location)
                self.leg_height = 0
                self.leg_offset_x = 0
                self.leg_offset_z = 0
                self.leg_in_air = 0
            elif 'dup' in buttons_pressed and self.leg_height < self.max_leg_height:
                self.leg_height += 1
            elif 'ddown' in buttons_pressed and self.leg_height > -self.max_leg_height:
                self.leg_height -= 1

            # Control movement in z direction
            if left_cy > 0 and self.leg_offset_z < self.max_offset:
                self.leg_offset_z += 1
            if left_cy < 0 and self.leg_offset_z > -self.max_offset:
                self.leg_offset_z -= 1

            # Control movement in the x direction. Positive x is towards the inside of the robot, so we must take leg_in_air into consideration
            if self.leg_in_air == 0:
                if left_cx > 0 and self.leg_offset_x < self.max_offset:
                    self.leg_offset_x += 1
                if left_cx < 0 and self.leg_offset_x > -self.max_offset:
                    self.leg_offset_x -= 1
            else:
                if left_cx > 0 and self.leg_offset_x > -self.max_offset:
                    self.leg_offset_x -= 1
                if left_cx < 0 and self.leg_offset_x < self.max_offset:
                    self.leg_offset_x += 1

            if self.leg_in_air == 1:
                air_leg = 1
            else:
                air_leg = 0
            (temp_x, temp_y, temp_z) = self.points[air_leg][self.current_substep]
            new_x = temp_x + self.leg_offset_x
            new_y = temp_y + self.leg_height
            new_z = temp_z + self.leg_offset_z
            self.points[air_leg][self.current_substep] = (new_x, new_y, new_z)


    # for now use the version form Mode. todo: write a version to take the sensor into consideration
    # def next_substep(self, sensor):
    #     pass


    # This method preps the transition to lifting up one leg.
    # The way it does this is by creating the first step for all the legs to reach the desired location.
    # This methods takes into consideration the leg that needs to be lifted, and updates the endpoints accordingly
    # Note: this method sets is_finished_step = false in order for the robot to consider this setup a step
    # Currently, move legs as if robot is a square, might update according to results
    def stand_to_3_legs(self):
        for i in range(4):
            (def_x, def_y, def_z) = Settings.default_with_offset[i]
            if self.leg_in_air == 1:
                (offset_x, offset_y, offset_z) = self.default_right_offset[i]
            else:
                (offset_x, offset_y, offset_z) = self.default_left_offset[i]
            self.end_points[i] = (def_x + offset_x, def_y + offset_y, def_z + offset_z)

        self.is_finished_step = False
