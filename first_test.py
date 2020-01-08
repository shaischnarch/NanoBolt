# first test python file

from Kinematics import Movement_display as display
from Kinematics.K_and_IK_calculations import *


start_point = (-35,-135,75)
end_point = (-55,-100,-75)
angles = calculate_movement(start_point,end_point,75,8)
display.draw_movement(angles)