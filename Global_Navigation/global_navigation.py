import cv2
from matplotlib import pyplot as plt
import numpy as np
import pyvisgraph as vg
from geopandas import GeoSeries
from shapely.geometry import Polygon, Point, LineString


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


def obstacles_to_polygons(obst):
    """
    Convert the points given by Vision functions, then convert them using Polygon (check made to know
    if the conversion is possible)
    :param obst:  list of points detected by the Vision related functions
    :return polygons: A geoserie elements containing all the obstacle
    """
    poly = []
    final = []
    for obstacle in obst:
        polygon = []
        for point in obstacle:
            polygon.append([point[0][0], point[0][1]])
        poly.append(polygon)
        # test to know if the obstacle can be converted
        if np.size(polygon) > 6:
            final.append(Polygon(polygon))
    polygons = GeoSeries(final)
    return polygons


def polygons_2_points(polygons):
    """
    Inverse of obstacles_to_polygons(): turn a set of polygons in points for vysgraph algorithm
    :param polygons: set of polygons
    :return: polygons in the form of points
    """
    polygon = []
    for pol in polygons:
        x, y = pol.exterior.coords.xy
        list_point = []
        for i in range(len(x)):
            list_point.append(vg.Point(x[i], y[i]))
        polygon.append(list_point)
    return polygon


def build_vis_graph(shapes, start, target):
    """
    Take the information extracted to compute the shortest path using visibility graphs algorithms
    :param shapes: List of all obstacles
    :param start: Starting position of the robot
    :param target: Goal identified
    :return: The shortest path to go to the goal from the initial position
    """
    startx = str(start[0])
    starty = str(start[1])

    start = vg.Point(startx, starty)
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


def draw_thymio(img, path):
    for i in range(len(path)):
        img = cv2.circle(img, (int(path[i][0]), int(path[i][1])), int(3), (0, 0, 255), 2)
    return img
