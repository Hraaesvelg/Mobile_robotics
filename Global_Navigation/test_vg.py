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
    final = []
    for obstacle in obst:
        polygon = []
        for point in obstacle:
            polygon.append([point[0][0], point[0][1]])
        poly.append(polygon)
        if np.size(polygon) > 6:
            final.append(Polygon(polygon))
    return final


def plot_map(polygons):
    for pol in polygons:
        plt.plot(*pol.exterior.xy)
    plt.show()


def mix_obstacles(polygons):
    # Create Geometric graph of the obstacles as given by the vision analysis
    g = GeoSeries(polygons)
    return g


def plot_geometric_data(g):
    g.plot('Reds')


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


start = vg.Point(10, 10)
goal = vg.Point(350, 550)

polys = [[vg.Point(0, 150), vg.Point(0, 100), vg.Point(250, 150), vg.Point(250,100)],
         [vg.Point(150, 400), vg.Point(150, 450), vg.Point(400, 400), vg.Point(400,450)]]





