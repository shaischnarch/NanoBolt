from Excution.All_executions import *


## function used to set robot to the zero position
def zero_position():
    execute_movement(0, (90, 90, 90))
    execute_movement(1, (90, 90, 90))
    execute_movement(2, (90, 90, 90))
    execute_movement(3, (90, 90, 90))
    return [(-25, -150, -75), (25, -150, 75), (25, -150, -75), (-25, -150, 75)]
