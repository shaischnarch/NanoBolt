# first test python file

from Calculations import Movement_display as display
from Calculations.All_calculations import *


start_point = (0,-100,0)
end_point = (50,-100,50)
angles = calculate_movement(start_point,end_point,50,8)
display.draw_movement(angles)
