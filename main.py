# Premade libraries
import cv2
import time
from matplotlib import pyplot as plt

# Homemade functions
import Vision.vision as vs
import robot as rbt
import Global_Navigation.global_navigation as glb
import Motion_Control.thymio_control as ctrl

# Main
# Data
margin = 50

# Initialisation
# Create an instance of our robot
thymio = rbt.RobotNav()
node = 0
# Start the video capture
cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture(1)

while cap.isOpened():
    img = vs.get_image(cap)
    ret, frame = cap.read()
    if thymio.get_state() == 0:
        thymio.initialisation_step(img, margin, True)
        thymio.increase_step()
        thymio.set_state(1)
    elif thymio.get_state() == 1:
        # We use the position determined by the camera
        position, st = vs.detect_start(frame, False, False)
        frame = glb.draw_thymio(frame, position)
        frame = glb.draw_path(frame, thymio.get_path())
        # We update the position of the robot
        if position is not None:
            thymio.test()
            thymio.update_position_cam(st)
        else:
            thymio.update_position_kalman()
        """
        # We check if the robot encounter an obstacle
            to be implemented
        """
        # We try to reach the next goal
        geometry = thymio.get_geometry()
        step = thymio.get_crt_step()
        path = thymio.get_path()
        ctrl.astolfi((geometry[0], geometry[1]), geometry[2], path[step], node)
        print(path[step])
        thymio.update_step_respo()

    elif thymio.get_state() == 2:
        ctrl.stop_motors(node)
        ctrl.leds_blink(node)
        break # Exits the code


    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()


