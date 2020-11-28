from Helper_directory import Settings
from Calculations.All_calculations import calculate_points, servo_angles, legIK
import numpy as np
import math
from Modes_directory.Mode_Class import Mode

## Mode inherited class , where the robot is stable while all his legs on the ground
#  In this mode the robot uses the imu sensor to keep its self standing level
#  To go into this mode: press SQUARE on the controller when in stand mode (Mode)
#  Controls: Left stick Y - change standing height
#            L1 - Reset standing height to default height
class Stable_4_legs(Mode):

    def __init__(self, current_legs_location):
        Mode.__init__(self, current_legs_location)
        self.pause_movement = False
        self.num_of_substeps = 1
        self.sensor_act = 1
        self.standing_height = 0  # an addition to the height that is controlled by the controller, negative is robot standing taller
        self.max_standing_height = 30  # the max height offset from the controller (in both up and down directions)


    def calculate_points(self):
        return


    # Controls the height of the robot using the left stick
    # todo: make transition back to standing_height = 0 smoother (robot has hard time, especially when he needs to get up)
    def controller_input(self, left_cx, left_cy, right_cx, right_cy, buttons_pressed):
        if 'square' in buttons_pressed:  # 'square' is actually L1 on the ps4 controller, bad controller library
            self.standing_height = 0
            return
        if left_cy > 0 and self.standing_height > -self.max_standing_height:
            self.standing_height -= 1
        if left_cy < 0 and self.standing_height < self.max_standing_height:
            self.standing_height += 1


    # Overwrite update_substep to do nothing
    def update_substep(self):
        return


    def prep_substep(self, sensor):
        self.Stable_4_legs_sensor_helper(sensor)
        for i in range(4):
            (offsetX, offsetY, offsetZ) = Settings.legs_offset[i]
            (sensor_offset1, sensor_offset2, sensor_offset3) = self.sensor_offset[i]
            try:
                x = self.default_x + offsetX + sensor_offset1
                y = self.default_y + offsetY + sensor_offset2 + self.standing_height
                z = self.default_z + offsetZ + sensor_offset3
                print((x,y,z))
                (theta1, theta2, theta3) = legIK(x, y, z)
                self.angles_servo[i] = servo_angles([(theta1, theta2, theta3)], i)
                self.current_legs_location[i] = (x, y, z)
            except:
                print('ERROR: Tried to move to impossible position')
                self.stop_movement = True



    def Stable_4_legs_sensor_helper(self, sensor):
        (euler1, euler2, euler3) = sensor.euler
        print((euler1, euler2, euler3))
        offsetsX = [0, 0, 0, 0]
        offsetsY = [0, 0, 0, 0]
        offsetsZ = [0, 0, 0, 0]

        if (abs(euler2) > self.sensor_act):
            offsetsY[0] -= np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))
            offsetsY[1] -= np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))
            offsetsY[2] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))
            offsetsY[3] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))

            offsetsZ[0] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))
            offsetsZ[1] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))
            offsetsZ[2] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))
            offsetsZ[3] += np.sign(euler2) * (1 + int(math.fabs(euler2) / 5))

        if (abs(euler3) > self.sensor_act):
            offsetsY[0] += np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))
            offsetsY[1] -= np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))
            offsetsY[2] -= np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))
            offsetsY[3] += np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))

            offsetsX[0] += np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))
            offsetsX[1] -= np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))
            offsetsX[2] -= np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))
            offsetsX[3] += np.sign(euler3) * (1 + int(math.fabs(euler3) / 5))

        for j in range(4):
            lst = list(self.sensor_offset[j])
            lst[0] += offsetsX[j]
            lst[1] += offsetsY[j]
            lst[2] += offsetsZ[j]
            self.sensor_offset[j] = tuple(lst)

