# Premade libraries
import cv2
import time
from matplotlib import pyplot as plt

# Homemade functions
import Vision.vision as vs
import Global_Navigation.robot as rbt
import Global_Navigation.global_navigation as glb
import Motion_Control.thymio_control as ctrl

# Main
# Data
cap = cv2.VideoCapture(0)
img = vs.get_image(cap)
img = "Vision/test_2.png"
margin = 20
simulation = 1
# Initialisation
# Create an instance of our robot
thymio = rbt.RobotNav()
# Start the video capture
cap = cv2.VideoCapture(0)
cursor = 0
while cap.isOpened():
    ret, frame = cap.read()
    if thymio.get_state() == 0:
        thymio.initialisation_step(img, margin, True)
        thymio.increase_step()
        thymio.set_state(1)
    elif thymio.get_state() == 1:
        # We use the position determined by the camera
        img = vs.get_image(cap)
        print('curent step is ' + str(cursor))
        time.sleep(0.1)
        if cursor > 20:
            thymio.set_state(2)
        cursor = cursor + 1

        position, st = vs.detect_start(frame, False, False)
        print(position)
        # thymio.update_position_cam(position)
        glb.draw_thymio(frame, position)
        cv2.imshow('video', frame)

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
    elif thymio.get_state() == 2:
        ctrl.stop_thymio()
        break


cap.release