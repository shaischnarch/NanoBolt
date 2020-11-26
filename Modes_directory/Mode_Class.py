from Helper_directory import Settings
from Helper_directory.Main_helper import move_to_stand
from Calculations.All_calculations import calculate_points
import math

## Main virtual class, all functional modes of the robot inherit this class
## starts by moving the robot to a base standing position
class Mode:

    def __init__(self, current_legs_location):
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

    def calculate_points(self):
        for leg_num in range(4):
            self.points[leg_num] = calculate_points(self.current_legs_location[leg_num], self.end_points[leg_num], self.heights[leg_num], self.num_of_substeps)
        self.current_substep = 0
        self.is_finished_step = False


    # check that its working
    def update_substep(self):
        self.current_substep += 1
        for leg_num in range(4):
            self.current_legs_location[leg_num] = points[leg_num][self.current_substep]
        if (self.current_substep >= self.num_of_substeps):
            self.is_finished_step = True
























