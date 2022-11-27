"""
Class robot used to accomplish global navigation
"""

import math
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import numpy as np


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

        self.x_img = None
        self.y_img = None
        self.theta_img = None

        # Global navigation
        self.path = None
        self.start = None
        self.goal = None
        self.next_step = None
        self.finished = False
        self.crt_stp = 0

    def initialize_starting_pos(self, cam_data):
        """
        Initialise the starting position of the robot
        from camera data
        :param cam_data: ((x_center, y_center),(x_front, y_front)): allows us to  vectorize Thymio position
        """
        if cam_data is not None:
            self.theta_img = math.atan2(cam_data[0][1] - cam_data[1][1], cam_data[0][0] - cam_data[1][0])
            self.x_img = cam_data[0][0]
            self.y_img = cam_data[0][1]

            self.x_kalman = self.x_img
            self.y_kalman = self.y_img
            self.theta_kalman = self.theta_img

            self.start = (self.x_img, self.y_img)
            """
            print(self.x_img, self.y_img)
            print(math.degrees(self.theta_img))
            print(cam_data)
            plt.scatter([cam_data[0][0], cam_data[1][0]], [cam_data[0][1], cam_data[1][1]], c = 'red')
            plt.show()
            """

    def update_position_cam(self, cam_data):
        """
        Update robot's position from camera data
        :param cam_data: ((x_center, y_center),(x_front, y_front)): allows us to  vectorize Thymio position
        """
        if cam_data is not None:
            self.theta_img = math.atan2(cam_data[0][1] - cam_data[1][1], cam_data[0][0] - cam_data[1][0])
            self.x_img = cam_data[0][0]
            self.y_img = cam_data[0][1]

    def set_goal(self, goal):
        """
        Set the goal's coordinates of our robot
        :param goal: (x,y) goal's coordinates
        """
        self.goal = goal

    def find_path_astar(self, matrix, show):
        """
        Find the shortest path from the starting point to the goal
        :param: grid with 1=free, 0 or less = obstacle / starting point/ ending point/ show : enable display of the map
        :return : list of coordinates corresponding to path
        """

        grid = Grid(matrix=matrix)
        start = grid.node(self.start[0], self.start[1])
        end = grid.node(self.goal[0], self.goal[1])

        finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
        path, runs = finder.find_path(start, end, grid)

        self.path = path

        if show:
            print(path)
            print('operations:', runs, 'path length:', len(path))
            print(grid.grid_str(path=path, start=start, end=end))

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

    def update_step_respo(self, case_width, tolerance, show):
        """
        Determine to which case of the map the robot belong
        :param case_width: width of the case which path(crt_stp) is the center
        :return:
        """
        next_step = self.path(self.crt_stp + 1)
        dist = math.sqrt((self.x_kalman - next_step[0]) ^ 2 + (self.y_kalman - next_step[1]) ^ 2)

        if dist < tolerance:
            if show:
                print(self.path(self.crt_stp), self.crt_stp)
            if self.crt_stp == len(self.path) - 1:
                self.finished = True
            else:
                self.crt_stp = self.crt_stp + 1

        return self.crt_stp
