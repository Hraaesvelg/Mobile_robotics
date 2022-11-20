"""
Main code to test our functions
"""

import robot as rbt
import thymio_control as ctrl
from tdmclient import ClientAsync, aw

Thym = rbt.RobotNav()
Thym.initialize_starting_pos(cam_data)
Thym.set_goal(goal)

while True:
    Thym.update_step_respo(case_width = 50, tolerance = 10, show = False)
    if not Thym.finished:
        ctrl.stop_motors(node)
        break

    angle = Thym.get_angle2goal()  # angle is btw [-pi, pi]
    sp_l = Thym.speed + angle
    sp_r = Thym.speed + angle

    ctrl.set_motor_speed(sp_r, sp_l, node)
