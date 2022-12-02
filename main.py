# Premade libraries
import cv2
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

while cap.isOpened():
    if thymio.get_state() == 0:
        thymio.initialisation_step(img, margin, True)
        thymio.increase_step()
        thymio.set_state(1)
    elif thymio.get_state() == 1:
        # We use the position determined by the camera
        if vs.get_image(cap) is not None:
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

    elif thymio.get_state() == 2:
        ctrl.stop_thymio()

