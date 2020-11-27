from Helper_directory import Settings
from Helper_directory.Main_helper import move_to_stand
from Calculations.All_calculations import calculate_points
import math


## Main virtual class, all functional modes of the robot inherit this class
#  starts by moving the robot to a base standing position
#  also known as stand
class Mode:

    def __init__(self, current_legs_location):
        self.stop_movement = False  # stops the current movement, can only be reset by moving back to standing position
        self.pause_movement = True
        self.default_x = Settings.default_x
        self.default_y = Settings.default_y
        self.default_z = Settings.default_z
        self.substep_delay = Settings.max_delay
        self.num_of_substeps = 32
        self.current_substep = 0
        self.is_finished_step = True
        self.heights = [0, 0, 0, 0]
        self.end_points = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
        self.points = [[], [], [], []]
        self.angles_servo = [[], [], [], []]
        self.sensor_offset = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
        self.current_legs_location = move_to_stand(current_legs_location)


    # Main function for planing each step step (Not Substep!).
    # Therefore it is called only after is_finished_step is True.
    # @Receives the controller inputs, and decides what the next step should be
    # Note: both this and .controller_input receive the controller input. This method decides if it should use the controller input to change the next STEP
    # while .controller_input decides if it should use the controller input to change the next SUBSTEP
    # @Returns nothing
    # *Important* this method does not decide to change mode, and should only decide what the next step should be in the SAME mode
    def plan_movement(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):
        pass

    # Main function for calculating all the substeps locations along the a step.
    # Therefore it is called only after plan_movement and it finishes setting up the new step.
    # @Receives: nothing
    # @Resets: is_finished_step = false, current_substep = 0
    # @Returns nothing
    def calculate_points(self):
        for leg_num in range(4):
            self.points[leg_num] = calculate_points(self.current_legs_location[leg_num], self.end_points[leg_num], self.heights[leg_num],
                                                    self.num_of_substeps)
        # self.current_substep = 0
        self.current_substep = 1  # EXPERIMENTAL, might need to be lowered back down to 0
        self.is_finished_step = False


    # Method for receiving updates in the middle of a step.
    # For example, can look at stick values and update walking speed
    # Also used to update pause_movement
    # @Receives the controller inputs
    # @Returns nothing
    def controller_input(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):
        pass


    # This method is called upon before executing the next substep.
    # When this method is called, the location of the next substep is already available in self.points[leg_num][self.current_substep]
    # Therefore this method uses the sensor to make realtime minor changes to the next substep.
    # It than calculates the actual values for the servos to execute the next substep
    # @updates: current_legs_location with the real time location
    def prep_substep(self, sensor):
        for leg_num in range(4):
            (point_x, point_y, point_z) = self.points[leg_num][self.current_substep]
            (offsetX, offsetY, offsetZ) = Settings.legs_offset[leg_num]
            (sensor_offset1, sensor_offset2, sensor_offset3) = self.sensor_offset[leg_num]
            try:
                x = point_x + offsetX + sensor_offset1
                y = point_y + offsetY + sensor_offset2
                z = point_z + offsetZ + sensor_offset3
                (theta1, theta2, theta3) = legIK(x, y, z)
                self.angles_servo[leg_num] = servo_angles([(theta1, theta2, theta3)], leg_num)
                self.current_legs_location[leg_num] = (x, y, z)
            except:
                print('ERROR: Tried to move to impossible position')
                self.stop_movement = True


    # Updates internal values for the next substep.
    # these include the current_substep
    # lastly, raises is_finished_step when step is complete
    #  todo: check that its working
    def update_substep(self):
        # for leg_num in range(4):
        #     self.current_legs_location[leg_num] = points[leg_num][self.current_substep]
        self.current_substep += 1
        if self.current_substep > self.num_of_substeps:  # There are num_of_substeps+1 indexes
            self.is_finished_step = True


    """ OLD Version. todo: if new one works, delete this
    # Updates internal values for the next substep.
    # these include the current_substep as well as the current_leg_location
    # lastly, raises is_finished_step when step is complete
    #  todo: check that its working
    def update_substep(self):
        self.current_substep += 1
        for leg_num in range(4):
            self.current_legs_location[leg_num] = points[leg_num][self.current_substep]
        if self.current_substep >= self.num_of_substeps:
            self.is_finished_step = True
    """