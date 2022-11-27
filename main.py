import cv2
from matplotlib import pyplot as plt
import numpy as np
import Vision.vision as vs
import Global_Navigation.robot as rbt
import Global_Navigation.test_vg as tst
import Global_Navigation.visibility_graph as gvg

import pyvisgraph as vg
import math
import geopandas as gpd
from geopandas import GeoSeries
from shapely.geometry import Polygon, Point, LineString


def point_2_polygons(obst):
    poly = []
    for obstacle in obst:
        polygon = []
        for point in obstacle:
            polygon.append(vg.Point(point[0][0], point[0][1]))
        poly.append(polygon)
    return poly


def get_path_lines(image, path):
    path_lines = []
    for i in range(len(path) - 1):
        path_lines.append([(int(path[i].x), int(path[i].y)), (int(path[i + 1].x), int(path[i + 1].y))])
    for line in path_lines:
        image = cv2.line(image, line[0], line[1], (255, 0, 255), 4)
    return image


def plot_geometric_data(g):
    g.plot('Reds')
    plt.show()


def obstacles_to_polygons(obst):
    # Convert this polygone list into a geometric set of polygons
    poly = []
    final = []
    for obstacle in obst:
        polygon = []
        for point in obstacle:
            polygon.append([point[0][0], point[0][1]])
        poly.append(polygon)
        if np.size(polygon) > 6:
            final.append(Polygon(polygon))
    g = GeoSeries(final)
    return g


def polygons_add_margin(g, margin):
    # Geometric graph of the obstacles with the margin
    g=g.buffer(margin,join_style=2)
    return g


def polygons_to_VisibilityGraph(g):
    # Visibility graph created from the geometric graph
    polygons = []
    for poly in g:
        x, y = poly.exterior.coords.xy
        polygon_vg = []
        for i in range(len(x)):
            polygon_vg.append(vg.Point(x[i], y[i]))
        polygons.append(polygon_vg)

    visgraph = vg.VisGraph()
    visgraph.build(polygons)

    return visgraph
"""
pts2 = obstacles_to_polygons(obstacles)
print(pts2)
plot_geometric_data(pts2)

pts2_marged = polygons_add_margin(pts2,30)
pts2_marged = pts2_marged[2:]
pts2_mrgd = GeoSeries([pts2_marged[2], pts2_marged[4]])

print(pts2_mrgd)
plot_geometric_data(pts2_mrgd)

vs_graph = polygons_to_VisibilityGraph(pts2_mrgd)



start = vg.Point(start[0][0], start[0][1])
target = vg.Point(target[0], target[1])
target = vg.Point(400, 600)

shortest = vs_graph.shortest_path(start, target)
path = shortest

print(path)

"""


# main
img = "Vision/cercle1.png"

thymio = rbt.RobotNav()
start, target, obstacles, size = vs.transmit_data(img, False)


print('the points found are: ')
points = point_2_polygons(obstacles)
poly = obstacles_to_polygons(obstacles)
poly = polygons_add_margin(poly, 0)

polygons = []
for pol in poly:
    x, y = pol.exterior.coords.xy
    polygon_vg = []
    for i in range(len(x)):
        polygon_vg.append(vg.Point(x[i], y[i]))
    polygons.append(polygon_vg)

print(polygons)



points = polygons

plot_geometric_data(poly)


start = vg.Point(start[0][0], start[0][1])
print(start)
#target = vg.Point(target[0], target[1])
target = vg.Point(400, 244)

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
