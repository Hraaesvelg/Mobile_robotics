"""
Main code to test our functions
"""
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import robot as rbt

Thym = rbt.RobotNav
matrix = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 1, 1, 0, 0, 1, 1, 1],
        [0, 0, 0, 1, 1, 0, 0, 1, 1, 1],
        [1, 1, 0, 1, 1, 1, 0, 0, 0, 0],
        [1, 0, 1, 1, 0, 1, 1, 1, 1, 0],
        [1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 1, 0, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 0]]

Thym.initialize_starting_pos(Thym, ((0,0),(0,0)))
Thym.set_goal(Thym, (5,9))

print(Thym.start)
print(Thym.goal)

Thym.find_path_astar(Thym, matrix, True)

