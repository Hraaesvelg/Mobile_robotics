"""
Main code to test our functions
"""

import robot as rbt
import thymio_control as ctrl
from tdmclient import ClientAsync, aw

Thym = rbt.RobotNav()
Thym.initialize_starting_pos(((0,0),(0,0)))
Thym.set_goal((5,9))

while True:
    Thym.update_step_respo(case_width = 50, tolerance = 10, show = False)
    if not Thym.finished:
        ctrl.stop_motors(node)
        break

    angle = Thym.get_angle2goal()  # angle is btw [-pi, pi]
    sp_l = Thym.speed + angle
    sp_r = Thym.speed + angle

    ctrl.set_motor_speed(sp_r, sp_l, node)



"""
while cap.isOpened():
    ret, frame = cap.read()
    if thymio.get_state() == 0:
        thymio.initialisation_step(img, 50, True)
        thymio.increase_step()
        thymio.set_state(1)
    elif thymio.get_state() == 1:
        # We use the position determined by the camera
        position, st = vs.detect_start(frame, False, False)
        print(position)
        # thymio.update_position_cam(position)
        glb.draw_thymio(frame, position)
        """
        """
        if img is not None:
            position = vs.detect_start(img, False, False)
            thymio.update_position_cam(position)
        # Without data from the camera we apply a kalman filter
        else:
            thymio.update_position_kalman()

        if ctrl.detect_obstacle():
            thymio.avoidance_procedure()

        while abs(thymio.get_angle2goal()) < 0.1:
            if thymio.get_angle2goal() < 0:
                ctrl.rotate('left', 50, 1)
            else:
                ctrl.rotate('right', 50, 1)

        ctrl.advance(50, 1)
        thymio.update_step_respo()
    """
    """
    elif thymio.get_state() == 2:
        ctrl.stop_thymio()
        break

    cv2.imshow('video', frame)

    cap.release
"""

