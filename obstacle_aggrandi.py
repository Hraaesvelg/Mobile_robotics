import cv2
from matplotlib import pyplot as plt
import Vision.vision as vs
import numpy as np

from geopandas import GeoSeries
from shapely.geometry import Polygon, Point, LineString




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




#cap = cv2.VideoCapture(0)
#vs.get_image(cap)

img = "Vision/cercle1.png"
#vs.transmit_data(img,False)
#

start, target, obstacle, size = vs.transmit_data(img, False)
poly = obstacles_to_polygons(obstacle)

#print(poly[1])
#x, y = poly[1].exterior.coords.xy
#print(len(x))
#print(x)
#print(y)
#print(len(poly[1]))

image = cv2.imread(img)
img_margin, contours = vs.draw_increase_obstacle(image, poly)

#print(contours_agr)
fig, (ax1, ax2) = plt.subplots(1, 2, sharey='row')
ax1.imshow(img_margin)

ax2.imshow(img_margin)


plt.show()