from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

matrix = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 1, 1, 0, 0, 1, 1, 1],
        [0, 0, 0, 1, 1, 0, 0, 1, 1, 1],
        [1, 1, 0, 1, 1, 1, 0, 0, 0, 0],
        [1, 0, 1, 1, 0, 1, 1, 1, 1, 0],
        [1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
        [1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [1, 0, 0, 1, 1, 1, 1, 1, 1, 0],
        [1, 1, 0, 0, 1, 1, 0, 1, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 1, 0, 0]]

grid = Grid(matrix=matrix)

start = grid.node(0, 0)
end = grid.node(5, 9)

finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
path, runs = finder.find_path(start, end, grid)
print(path)
print('operations:', runs, 'path length:', len(path))
print(grid.grid_str(path=path, start=start, end=end))











def find_path_astar(matrix, start_point, goal_point):
    """
    Find the shortest path from the starting point to the goal

    input: grid with 1=free, 0 or less = obstacle / starting point/ ending point
    output : list of coordinates cooresponding to path

    """
    grid = Grid(matrix=matrix)
    start = grid.node(start_point[0], start_point[1])
    end = grid.node(goal_point[0], goal_point[1])

    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(start, end, grid)

    return path

