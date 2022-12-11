import cv2
from matplotlib import pyplot as plt
import pyvisgraph as vg


def get_path_lines(image, path):
    """
    Draw the lines on our CV2 image
    :param image: image we want to draw onto
    :param path: generated path using the vysgraph function
    :return: the modified image
    """
    path_lines = []
    for i in range(len(path) - 1):
        path_lines.append([(int(path[i].x), int(path[i].y)), (int(path[i + 1].x), int(path[i + 1].y))])
    for line in path_lines:
        image = cv2.line(image, line[0], line[1], (255, 0, 255), 4)
    return image


def plot_geometric_data(map, color='Blues', show=True):
    """
    Plot obstacles using geoserie plot function
    :param show:  Boolean to plot the geoserie or not
    :param color: panel of colors used by our plot function
    :param map: geoserie we want to plot
    :return: plot the map
    """
    map.plot(color)
    if show:
        plt.show()


def build_vis_graph(shapes, start, target):
    """
    Take the information extracted to compute the shortest path using visibility graphs algorithms
    :param shapes: List of all obstacles
    :param start: Starting position of the robot
    :param target: Goal identified
    :return: The shortest path to go to the goal from the initial position
    """
   
    start = vg.Point(str(start[0]), str(start[1]))
    target = vg.Point(str(target[0]), str(target[1]))
    vgPoints = []

    for i in range(len(shapes)):
        temp = []
        for j in range(len(shapes[i])):
            temp.append(vg.Point(shapes[i][j][0], shapes[i][j][1]))
        temp.append(vg.Point(shapes[i][0][0], shapes[i][0][1]))
        vgPoints.append(temp)

    g = vg.VisGraph()
    g.build(vgPoints)

    shortest_path = g.shortest_path(start, target)
    return shortest_path, shapes


def draw_path(img, shortest):
    """
    Draw the path build by build_vis_graph() and print it on the image using CV2 functions
    :param img: The Image we want to modify
    :param shortest: The shortest path constructed previously
    :return: The modified image
    """
    for i in range(len(shortest ) -1):
        cv2.line(img, (int(shortest[i].x), int(shortest[i].y)), (int(shortest[ i +1].x), int(shortest[i+1].y)),
                 (255, 0, 0), thickness=5)
    return img


def draw_thymio(img, path, mode):
    """
    Draw all the position recorded in the path on the image
    :param img: The Image we want to modify
    :param path: A list of all the previous position of the robot
    :param mode: If the input mode is 'kalman' draw all position in a different color. If not draw in red
    :return:
    """
    for i in range(len(path)):
        if mode == 'kalman':
            img = cv2.circle(img, (int(path[i][0]), int(path[i][1])), int(3), (0, 255, 0), 2)
        else:
            img = cv2.circle(img, (int(path[i][0]), int(path[i][1])), int(3), (0, 0, 255), 2)
    return img
