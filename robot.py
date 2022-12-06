"""
Class robot used to accomplish global navigation
"""
# Premade libraries
import math
import numpy as np
import cv2
from matplotlib import pyplot as plt

# Homemade functions

import Global_Navigation.global_navigation as glb
import Vision.vision as vs
#import Filtering.KalmanFilter as klm


class RobotNav:
    def __init__(self):
        """
        Initializing robot class to enable path_finding
        """
        # Robot geometry
        self.radius = 55
        self.length = 110
        self.width = 110
        self.dist_wheel = 95
        self.speed = 50

        self.x_kalman = None
        self.y_kalman = None
        self.theta_kalman = None

        self.x = None
        self.y = None
        self.theta = None
        self.real_path = []
        self.real_path_karman = []

        # Global navigation
        self.path = None
        self.start = None
        self.goal = None
        self.next_step = None
        # state ==> 0 not initialized/ 1=moving to target/ 2=finished
        self.state = 0
        self.crt_stp = 0

    def initialize_starting_pos(self, cam_data, center):
        """
        Initialise the starting position of the robot
        from camera data
        :param center:
        :param cam_data: ((x_center, y_center),(x_front, y_front)): allows us to  vectorize Thymio position
        """
        if cam_data is not None:
            self.theta = math.atan2(cam_data[0][1] - cam_data[1][1], cam_data[0][0] - cam_data[1][0])
            self.x = center[0]
            self.y = center[1]

            self.x_kalman = self.x
            self.y_kalman = self.y
            self.theta_kalman = self.theta

            self.start = (self.x, self.y)

    def update_position_cam(self, cam_data):
        """
        Update robot's position from camera data
        :param cam_data: ((x_center, y_center),(x_front, y_front)): allows us to  vectorize Thymio position
        """
        if cam_data is not None:
            self.theta = math.atan2(cam_data[0][1] - cam_data[1][1], cam_data[0][0] - cam_data[1][0])
            self.x = cam_data[0][0]
            self.y = cam_data[0][1]
            self.real_path.append([self.x, self.y])

        


    def update_position_kalman(self):
        x_est, P_est = klm.kalman_filter()
        self.x_kalman = x_est[0]
        self.y_kalman = x_est[1]

        self.theta = self.theta_kalman
        self.x = self.x_kalman
        self.y = self.y_kalman

        self.real_path_karman.append([self.x_kalman, self.y_kalan])

    def set_goal(self, goal):
        """
        Set the goal's coordinates of our robot
        :param goal: (x,y) goal's coordinates
        """
        self.goal = goal

    def get_angle2goal(self):
        """
        Compute the angle between the curent orientation of the tymio and its next goal
        :return: beta the angle btw goal/crt orientation
        """
        if self.start is not None and self.goal is not None:
            goal = self.path[self.crt_stp + 1]
            beta = math.atan2(goal[1] - self.y_kalman, goal[0] - self.x_kalman)

            if beta - self.theta_kalman < -np.pi:
                beta = -(beta - self.theta_kalman + np.pi)
            elif beta - self.theta_kalman > np.pi:
                beta = -(beta - self.theta_kalman - np.pi)
            else:
                beta = beta - self.theta_kalman
            return beta

    def update_step_respo(self, tolerance,i, show):
        """
        Determine to which case of the map the robot belong
        :param show:
        :param tolerance:
        :return:
        """
       
        next_step = self.path[i]
       

        dist = math.sqrt(int(self.x - next_step.x)**2 + int(self.y - next_step.y)**2)
        print("diiiiiiiiiiiiiiiiiiistttt")
        if dist < tolerance:
            print("innnnnnnnnnnnnnnnn")
            i = i + 1
            if i == len(self.path):
                self.set_state(2)
            if show:
                 print(self.path(self.crt_stp), self.crt_stp)
            # if self.crt_stp == len(self.path) - 1:
            #     self.set_state(2)
            # else:
            #     self.set_state(1)
            #     self.increase_step()
        return i

    def initialisation_step(self, img, margin, show=False):
        self.state = 0
        #print("path1")
        start, target, shapes, size, start_points = vs.transmit_data(img, show, margin)
        #print("path2")
        # Initialize the starting position
        self.initialize_starting_pos(start_points, start)
        #print("path3")
        # Set the Goal of our robot
        self.set_goal(target)
        # Compute and assign the shortest path
        #print("path4")
        shortest = glb.build_vis_graph(shapes, start, target)
        self.path = shortest
        #print("path")
        #print(shortest)

        if show:
            #img = cv2.imread(img)
            img = glb.draw_path(img, shortest)
            plt.imshow(img)
            plt.show()

    def get_crt_step(self):
        return self.crt_stp

    def increase_step(self):
        self.crt_stp = self.crt_stp + 1

    def get_state(self):
        return self.state

    def set_state(self, state):
        print('state updated')
        self.state = state

    def get_path(self):
        return self.path

    def get_geometry(self):
        return self.x, self.y, self.theta

    def get_real_path(self):
        return self.real_path

    def get_real_path_kalman(self):
        return self.real_path_karman
