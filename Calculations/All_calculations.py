# functions for calculating leg movements and positions

from mpl_toolkits import mplot3d
import numpy as np
from math import *

# robots size constants
l1 = 25
l2 = 20
l3 = 80
l4 = 80


## main step calculation method.
# receives the start location of the step and end location (in (x,y,z))
# the function also receives the height of the step as height
# also receives the number of substeps (for n points there are n-1 substeps)
# returns an array with all the angles for each substep in order to complete the step
def calculate_movement(start_p, end_p, height, num_of_substeps):
    angles = []

    # finds all the end points for all the substeps
    # (INCLUDES ANGLES FOR THE STARTING POSITION, LEG DOESNT ACTUALLY NEED TO MOVE HERE)
    points = calculate_points(start_p, end_p, height, num_of_substeps)
    for i in range((num_of_substeps + 1)):
        (temp_x, temp_y, temp_z) = points[i]
        temp_angles = legIK(temp_x, temp_y, temp_z)
        angles.append(temp_angles)
    return angles


## helper step calculation method.
# receives the start location of the step and end location (in (x,y,z))
# the function also receives the height of the step as height (calculations shown in paper)
# also receives the number of substeps (for n points there are n-1 substeps)
# returns an array with all the points along the step (including the starting point) in total (num_of_substeps + 1)
def calculate_points(start_p, end_p, height, num_of_substeps):
    points = []
    (x1, y1, z1) = start_p
    (x2, y2, z2) = end_p

    # calculate each axis differential substep length
    dx = (x2 - x1) / num_of_substeps
    dy = (y2 - y1) / num_of_substeps
    dz = (z2 - z1) / num_of_substeps

    # add all the points starting from i=0 up to i=num_of_substeps (endpoint)
    for i in range((num_of_substeps + 1)):
        di = i / num_of_substeps
        temp_x = x1 + (dx * i)
        temp_y = y1 + (dy * i) + (-4 * height * di * di + 4 * height * di)
        temp_z = z1 + (dz * i)
        points.append((temp_x, temp_y, temp_z))
    return points


def legIK(x, y, z):
    """
    x/y/z=Position of the Foot in Leg-Space

    F=Length of shoulder-point to target-point on x/y only
    G=length we need to reach to the point on x/y
    H=3-Dimensional length we need to reach
    """

    F = sqrt(x ** 2 + y ** 2 - l1 ** 2)
    G = F - l2
    H = sqrt(G ** 2 + z ** 2)

    theta1 = -atan2(y, x) - atan2(F, -l1)

    D = (H ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4)
    theta3 = acos(D)

    theta2 = atan2(z, G) - atan2(l4 * sin(theta3), l3 + l4 * cos(theta3))

    return (theta1, theta2, theta3)


def calcLegPoints(angles):
    (theta1, theta2, theta3) = angles
    theta23 = theta2 + theta3

    T0 = np.array([0, 0, 0, 1])
    T1 = T0 + np.array([-l1 * cos(theta1), l1 * sin(theta1), 0, 0])
    T2 = T1 + np.array([-l2 * sin(theta1), -l2 * cos(theta1), 0, 0])
    T3 = T2 + np.array([-l3 * sin(theta1) * cos(theta2), -l3 * cos(theta1) * cos(theta2), l3 * sin(theta2), 0])
    T4 = T3 + np.array([-l4 * sin(theta1) * cos(theta23), -l4 * cos(theta1) * cos(theta23), l4 * sin(theta23), 0])

    return np.array([T0, T1, T2, T3, T4])




## receives angles array (each angle is (theta1,theta2,theta3)) in radians from the IK_calculations
# this function than converts said angles into degrees than does the necessary conversions
# to get the actual physical servo angles necessary to move the legs into the correct position and returns said angles
def servo_angles(angles_rad, side):
    angles_deg = convert_angles(angles_rad)
    angles_servo = []
    for i in range(len(angles_deg)):
        (temp_1, temp_2, temp_3) = angles_deg[i]
        if (side == 'left'):
            temp_1 = 180 - (temp_1 + 90)
            temp_2 = 180 - (temp_2 + 90)
            temp_3 = 180 - temp_3
        else:
            ######################### need to add ###########################
            pass
        ##see paper for the conversion explanation
        angles_servo.append((temp_1, temp_2, temp_3))
    return angles_servo


# convert angles array form radians to degrees
def convert_angles(angles_rad):
    angles_deg = []
    for i in range(len(angles_rad)):
        (temp_1, temp_2, temp_3) = angles_rad[i]
        temp_1 = degrees(temp_1)
        temp_2 = degrees(temp_2)
        temp_3 = degrees(temp_3)
        ##math.degrees(radian) converts into degrees
        angles_deg.append((temp_1, temp_2, temp_3))
    return angles_deg
