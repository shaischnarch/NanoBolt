from Helper_directory import Settings
from Helper_directory.Main_helper import move_to_stand
from Calculations.All_calculations import calculate_points,servo_angles, legIK
import math


## Main virtual class, all functional modes of the robot inherit this class
#  starts by moving the robot to a base standing position
#  also known as stand
class Mode:

    def __init__(self, current_legs_location):
        self.stop_movement = False  # stops the current movement, can only be reset by moving back to standing position
        self.pause_movement = True
        self.substep_delay = Settings.max_delay
        self.num_of_substeps = 32
        self.current_substep = 0
        self.is_finished_step = True
        self.heights = [0, 0, 0, 0]
        self.end_points = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
        self.points = [[], [], [], []]  # Semi-ideal realm!!!
        self.angles_servo = [[], [], [], []]
        self.sensor_offset = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
        self.controller_offset = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
        self.offsets = [(0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]
        self.semi_ideal_current_pos, self.current_legs_location = move_to_stand(current_legs_location)


    # Main function for planing each step step (Not Substep!).
    # Each step is calculated in the semi-ideal realm!
    # Therefore it is called only after is_finished_step is True.
    # @Receives the controller inputs, and decides what the next step should be
    # Note: both this and .controller_input receive the controller input. This method decides if it should use the controller input to change the next STEP
    # while .controller_input decides if it should use the controller input to change the next SUBSTEP
    # @Returns nothing
    # *Important* this method does not decide to change mode, and should only decide what the next step should be in the SAME mode
    def plan_movement(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):
        pass

    # Main function for calculating all the substeps locations along the a step.
    # Semi - ideal realm!!
    # Therefore it is called only after plan_movement and it finishes setting up the new step.
    # @Receives: nothing
    # @Resets: is_finished_step = false, current_substep = 0
    # @Returns nothing
    def calculate_points(self):
        for leg_num in range(4):  # todo: change self.current_legs_location to the end of points (the ideal location of the start of the step), and make sure it works
            self.points[leg_num] = calculate_points(self.semi_ideal_current_pos[leg_num], self.end_points[leg_num], self.heights[leg_num],
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
    # When this method is called, the location of the next semi-ideal substep is already available in self.points[leg_num][self.current_substep]
    # Therefore this method uses the sensor to make realtime minor changes to the next substep - transfer to physical/dynamic realm
    # It than calculates the actual values for the servos to execute the next substep
    # @updates: current_legs_location with the real time location
    # todo: might be a cause of error, debug later
    def prep_substep(self, sensor):
        self.__sum_offsets()
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



    # Method for summing all the internal offsets into offsets
    # @Returns nothing
    # @Updates self.offsets
    def __sum_offsets(self):
        for leg_num in range(4):
            (sensor_offset_x, sensor_offset_y, sensor_offset_z) = self.sensor_offset[leg_num]  # Offsets dictated by the sensor
            (controller_offset_x, controller_offset_y, controller_offset_z) = self.controller_offset[leg_num]  # Offsets set by the user with the controller - for example raise robot height
            self.offsets[leg_num] = (sensor_offset_x + controller_offset_x, sensor_offset_y + controller_offset_y, sensor_offset_z + controller_offset_z)


    # Updates internal values for the next substep.
    # these include the current_substep
    # lastly, raises is_finished_step when step is complete
    #  todo: check that its working
    def update_substep(self):
        self.current_substep += 1
        if self.current_substep > self.num_of_substeps:  # There are num_of_substeps+1 indexes
            self.is_finished_step = True
