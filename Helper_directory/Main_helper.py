from Excution.All_executions import *


## function used to set robot to the zero position
def zero_position():
    #execute_movement(0, (90, 135, 90))
    #execute_movement(1, (90, 80, 60))
    #execute_movement(2, (90, 45, 90))
    #execute_movement(3, (90, 100, 120))
    execute_movement(0, (90, 90, 90))
    execute_movement(1, (90, 90, 90))
    execute_movement(2, (90, 90, 90))
    execute_movement(3, (90, 90, 90))
    return [(-25, -150, -40), (25, -150, 40), (25, -150, -40), (-25, -150, 40)]
