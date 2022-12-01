# Premade libraries
import cv2
from matplotlib import pyplot as plt

# Homemade functions
import Vision.vision as vs
import Global_Navigation.robot as rbt
import Global_Navigation.global_navigation as glb


# main
img = "Vision/cercle1.png"

thymio = rbt.RobotNav()
start, target, shapes, size = vs.transmit_data(img, False)

shortest = glb.build_vis_graph(shapes, start, target)

img = cv2.imread("Vision/cercle1.png")
img = glb.draw_path(img, shortest)

plt.imshow(img)
plt.show()
