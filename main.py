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


    cv2.imshow('frame', frame)
    if cv2.waitKey(1) == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()


