import cv2
from matplotlib import pyplot as plt
import numpy as np
import Vision.vision as vs
import Global_Navigation.robot as rbt
import pyvisgraph as vg
from geopandas import GeoSeries
from shapely.geometry import Polygon, Point, LineString
# import main_bis as mb

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


def plot_geometric_data(map, color = 'Blues', show = True):
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


def secure_path(obstacle, margin):
    """
    Use this function to add a security margin between the obstacle and the path of the robot
    :param obstacle:
    :param margin: the margin by which the polygon is enlarged
    :return: the enlarged obstacles
    """
    obstacle=obstacle.buffer(margin, join_style=2)
    return obstacle


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



# main
img = "Vision/cercle1.png"

thymio = rbt.RobotNav()
start, target, obstacles, size = vs.transmit_data(img, False)
print(obstacles)

print('the points found are: ')

poly = obstacles_to_polygons(obstacles)
poly = secure_path(poly, 30)
points = polygons_2_points(poly)

plot_geometric_data(poly)


start = vg.Point(start[0][0], start[0][1])
print(start)
target = vg.Point(target[0], target[1])
target = vg.Point(350, 244)

g = vg.VisGraph()
print(g)
g.build(points)


shortest = g.shortest_path(start, target)
path = shortest


image = cv2.imread(img)

# Window name in which img is displayed
window_name = 'Image'


image = get_path_lines(image, path)
# Displaying the img
cv2.imshow(window_name, image)
cv2.waitKey(0)
