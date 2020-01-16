# display movement using matplotlib

from mpl_toolkits import mplot3d
import numpy as np
from math import *
import matplotlib.pyplot as plt
from Kinematics.K_and_IK_calculations import *
from Legs_controll import *


## main function for drawing a step and all its substeps
# receives angles array from calculate_movement in order to draw them and the speculated movement
def draw_movement(angles):
    # setup the figure
    ax_lst = setupUI(150)

    # add starting points and endpoints to both graphs
    # green is starting point and red is end point
    start_leg_points = calcLegPoints(angles[0])
    end_leg_points = calcLegPoints(angles[len(angles) - 1])
    drawLeg(ax_lst[0], start_leg_points, 'go')
    drawLeg(ax_lst[1], start_leg_points, 'go')
    drawLeg(ax_lst[0], end_leg_points, 'ro')
    drawLeg(ax_lst[1], end_leg_points, 'ro')

    # add middle points to right graph in yellow
    # also add speculated movement to left graph
    old_leg_points = start_leg_points
    for i in range(1, (len(angles) - 1)):
        new_leg_points = calcLegPoints(angles[i])
        drawLeg(ax_lst[1], new_leg_points, 'yo')
        ax_lst[0].plot([old_leg_points[4][0], new_leg_points[4][0]], [old_leg_points[4][2], new_leg_points[4][2]],
                       [old_leg_points[4][1], new_leg_points[4][1]], 'b')
        old_leg_points = new_leg_points

    ax_lst[0].plot([old_leg_points[4][0], end_leg_points[4][0]], [old_leg_points[4][2], end_leg_points[4][2]],
                   [old_leg_points[4][1], end_leg_points[4][1]], 'b')

    print_angles(servo_angles(angles))
    plt.show()

    return ax_lst


## setups the figure and ui for the leg movement
def setupUI(limit):
    ax_lst = []
    fig = plt.figure()
    for i in range(2):
        ax_lst.append(fig.add_subplot(1, 2, (i + 1), projection="3d"))
        ax_lst[i].set_xlim(-limit, limit)
        ax_lst[i].set_ylim(-limit, limit)
        ax_lst[i].set_zlim(-limit, limit)
        ax_lst[i].set_xlabel("X")
        ax_lst[i].set_ylabel("Z")
        ax_lst[i].set_zlabel("Y")
        ax_lst[i].view_init(elev=20., azim=135)

    ax_lst[0].set_title("start and end positions")
    ax_lst[1].set_title("substeps")
    return ax_lst


## draws the leg represented by p into ax with end_color as the color of the end of the foot
# (followed by o for example 'ro' for red)
def drawLeg(ax, p, end_color):
    ax.plot([p[0][0], p[1][0], p[2][0], p[3][0], p[4][0]],
            [p[0][2], p[1][2], p[2][2], p[3][2], p[4][2]],
            [p[0][1], p[1][1], p[2][1], p[3][1], p[4][1]], 'k-', lw=3)
    ax.plot([p[0][0]], [p[0][2]], [p[0][1]], 'bo', lw=2)
    ax.plot([p[4][0]], [p[4][2]], [p[4][1]], end_color, lw=2)


# prints to the terminal the angles of theta 1,2,3
def print_angles(angles):
    for i in range(len(angles)):
        (temp1, temp2, temp3) = angles[i]
        print("position number {}: /t theta1: {}  theta2: {}  theta3: {}".format(i, temp1, temp2, temp3))
