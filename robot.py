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
import Filtering.KalmanFilter as klm
import Motion_Control.thymio_control as ctrl


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

        # Coordinates of the two circles on the Thymio
        self.center_front = None
        self.center_back = None
        self.middle = None
        self.orientation = None

        # Values returned by the kalman filter
        self.x_kalman = None
        self.y_kalman = None
        self.theta_kalman = None
        self.path_kalman = []

        # Values we got from the picture
        self.x_img = None
        self.y_img = None
        self.theta_img = None
        self.path_img = []

        # Velocity in the x and y direction
        self.vx = None
        self.vy = None

        # Global navigation
        self.path = None
        self.start = None
        self.goal = None
        self.next_step = None

        # state ==> 0 not initialized/ 1=moving to target/ 2=finished
        self.state = 0
        self.crt_stp = 0

    # front is a boolean set to true if we want the front center and to false when we want the back center
    def get_thymio_values(self, img, front, node, client):

        x = 0
        y = 0
        vx = 0
        vy = 0
        start = vs.detect_start1(img, show=False, begin=False)

        self.center_front, self.center_back = start[1][0], start[1][1]
        thymio_coordinates = (self.center_front + self.center_back) / 2
        speed = (ctrl.get_motors_speed(node, client)[0] + ctrl.get_motors_speed(node, client)[1]) / 2 * SPEED_COEFF

        if start[2] is not None:
            detection = True
            if front:
                x = self.center_front[0]
                y = self.center_front[1]
                vx = speed * math.cos(self.theta_img)
                vy = speed * math.sin(self.theta_img)
                return x, y, vx, vy, detection
            else:
                x = self.center_back[0]
                y = self.center_back[1]
                return x, y

        else:
            detection = False

        return x, y, vx, vy, detection

    def set_last_position(self, front, back):
        front = np.array(front)
        back = np.array(back)
        self.center_front = front
        self.center_back = back
        self.middle = (front+back)/2
        self.orientation = self.compute_orientation(front, back)
        return 0

    # Return orientation compared to x-axis between pi and -pi
    def compute_orientation(self, front, back):
        x = front[0] - back[0]
        y = front[1] - back[1]
        orientation = math.atan2(y, x)
        return orientation

    def initialize_starting_pos(self, cam_data, center):
        """
        Initialise the starting position of the robot
        from camera data
        :param center:
        :param cam_data: ((x_center, y_center),(x_front, y_front)): allows us to  vectorize Thymio position
        """
        if cam_data is not None:
            self.theta_img = math.atan2(cam_data[0][1] - cam_data[1][1], cam_data[0][0] - cam_data[1][0])
            self.x_img = center[0]
            self.y_img = center[1]

            self.x_kalman = self.x_img
            self.y_kalman = self.y_img
            self.theta_kalman = self.theta_img

            self.start = (self.x_img, self.y_img)

    def update_position_cam(self, cam_data,node, client):
        """
        Update robot's position from camera data
        :param cam_data: ((x_center, y_center),(x_front, y_front)): allows us to  vectorize Thymio position
        """
        if cam_data is not None:
            self.theta_img = math.atan2(cam_data[0][1] - cam_data[1][1], cam_data[0][0] - cam_data[1][0])
            self.x_img = cam_data[0][0]
            self.y_img = cam_data[0][1]
            print(cam_data[1])
            print(cam_data[0])
            self.set_last_position(cam_data[1], cam_data[0])
            self.path_img.append(self.middle)
            self.vx, self.vy = ctrl.get_motors_speed(node, client)

    def update_position_kalman(self, node, client, detection):
        [x_front, y_front] = self.center_front
        [x_back, y_back] = self.center_back
        vx, vy = ctrl.get_motors_speed(node, client)

        dvx, dvy = vx - self.vx, vy - self.vy
        p_est = [1000 * np.eye(4)]

        x_est_front, p_est_front = klm.kalman_filter(0, 0, 0, 0, [x_front, y_front, vx, vy], p_est, dvx, dvy, False)
        x_est_back, p_est_back = klm.kalman_filter(0, 0, 0, 0, [x_back, y_back, vx, vy], p_est, dvx, dvy, False)

        self.set_last_position(x_est_front, x_est_back)
        self.path_kalman.append(self.middle)
        self.vx, self.vy = ctrl.get_motors_speed(node, client)

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
            goal = self.path(self.crt_stp + 1)
            beta = math.atan2(goal[1] - self.y_kalman, goal[0] - self.x_kalman)

            if beta - self.theta_kalman < -np.pi:
                beta = -(beta - self.theta_kalman + np.pi)
            elif beta - self.theta_kalman > np.pi:
                beta = -(beta - self.theta_kalman - np.pi)
            else:
                beta = beta - self.theta_kalman
            return beta

    def update_step_respo(self, tolerance, show):
        """
        Determine to which case of the map the robot belong
        :param show:
        :param tolerance:
        :return:
        """
        next_step = self.path(self.crt_stp)
        dist = math.sqrt((self.x_kalman - next_step[0]) ^ 2 + (self.y_kalman - next_step[1]) ^ 2)

        if dist < tolerance:
            if show:
                print(self.path(self.crt_stp), self.crt_stp)
            if self.crt_stp == len(self.path) - 1:
                self.state = 2
            else:
                self.crt_stp = self.crt_stp + 1
        return self.crt_stp

    def initialisation_step(self, img, margin, show=False):
        self.state = 0
        start, target, shapes, size, start_points = vs.transmit_data(img, show, margin)
        # Initialize the starting position
        self.initialize_starting_pos(start_points, start)
        # Set the Goal of our robot
        self.set_goal(target)
        # Compute and assign the shortest path
        shortest = glb.build_vis_graph(shapes, start, target)
        self.path = shortest
        print(shortest)
        plt.imshow(img)
        if show:
            print(type(img))
            img = cv2.imread(img)
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
        self.state = state

    def avoidance_procedure(self):
        print('Currently avoiding the obstacle')

    def get_path(self, type):
        if type == 'kalman':
            return self.path_kalman
        elif type == 'img':
            return self.path_img
        else:
            return self.path

    def get_geometry(self):
        return self.middle, self.orientation

