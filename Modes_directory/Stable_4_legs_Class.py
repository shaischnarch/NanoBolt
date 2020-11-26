from Modes_directory.Mode_Class import Mode

## Mode inherited class , where the robot is stable while all his legs on the ground
class Stable_4_legs(Mode):

    def __init__(self, current_legs_location):
        Mode.__init__(self, current_legs_location)
        self.pause_movement = False
        self.num_of_substeps = 1
        self.sensor_act = 1

    def plan_movement(self,  ds4):
        pass

    def calculate_points(self):
        return

    def update_substep(self):
        return

    def next_substep(self, sensor):
        self.Stable_4_legs_sensor_helper(sensor)
        for i in range(4):
            (offset1, offset2, offset3) = Settings.legs_offset[i]
            if i > 2:
                offset3 = offset3 - 10
            (sensor_offset1, sensor_offset2, sensor_offset3) = self.sensor_offset[i]
            (theta1, theta2, theta3) = legIK(self.default_x + offset1 + sensor_offset1, self.default_y + offset2 + sensor_offset2,
                                             self.default_z + offset3 + sensor_offset3)
            self.current_legs_location[i] = (self.default_x + offset1 + sensor_offset1, self.default_y +offset2+ sensor_offset2
                                            , self.default_z +offset3+ sensor_offset3)
            self.angles_servo[i] = servo_angles([(theta1, theta2, theta3)], i)


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

