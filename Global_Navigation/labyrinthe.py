# generates a square maze (size*size) with the binary tree technique
import numpy as np

def carve_maze(grid: np.ndarray, size: int) -> np.ndarray:
    output_grid = np.empty([size * 3, size * 3], dtype=str)
    output_grid[:] = '#'
    i = 0
    j = 0
    while i < size:
        w = i * 3 + 1
        while j < size:
            k = j * 3 + 1
            toss = grid[i, j]
            output_grid[w, k] = ' '
            if toss == 0 and k + 2 < size * 3:
                output_grid[w, k + 1] = ' '
                output_grid[w, k + 2] = ' '
            if toss == 1 and w - 2 >= 0:
                output_grid[w - 1, k] = ' '
                output_grid[w - 2, k] = ' '

            j = j + 1

        i = i + 1
        j = 0

    return output_grid


def preprocess_grid(grid: np.ndarray, size: int) -> np.ndarray:
    # fix first row and last column to avoid digging outside the maze external borders
    first_row = grid[0]
    first_row[first_row == 1] = 0
    grid[0] = first_row
    for i in range(1, size):
        grid[i, size - 1] = 1
    return grid

def get_grid(size, n, p):
    grid = np.random.binomial(n, p, size=(size, size))
    processed_grid = preprocess_grid(grid, size)
    return(processed_grid)

def inverse_grid(grid, size):
    for lin in range(size) :
        for col in range(size):
            if grid[lin, col] == 1:
                grid[lin, col] = 0
            elif grid[lin, col] == 0:
                grid[lin, col] = 1
    return grid

def display_laby (grid, size):
    output = carve_maze(grid, size)
    for elm in output:
        print(" ".join(elm))