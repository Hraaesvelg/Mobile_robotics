from matplotlib import pyplot as plt
import numpy as np
import cv2
import math




def get_image(cap):
    result, image = cap.read()

    if result:
        cv2.imwrite("Vision/test_2.png", image)
        return image
    else:
        print("no img read")
        return None


def detect_start1(image, begin=True):  #detect start coordinates by using Hough circles method
    img = image.copy()
    points = []
    rayon = []

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    gray = cv2.medianBlur(gray, 5)
    gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY, 11, 3.5)

    # Remove noise
    kernel = np.ones((3, 3), np.uint8)
    gray = cv2.erode(gray, kernel, iterations=1)
    gray = cv2.dilate(gray, kernel, iterations=1)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=25, param2=25, minRadius=10, maxRadius=50)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")

        for (x, y, r) in circles:
            cv2.circle(img, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(img, (x - int(1.4 * r), y - int(1.4 * r)), (x + int(1.4 * r), y + int(1.4 * r)),(255, 255, 255), -1)
            pos = (int(x), int(y))
            points.append(pos)
            rayon.append(r)

    test_detect = False # will be useful for kalman filter, false if thymio position not detected
    center1 = 0
    center2 = 0
    start_coordinates = 0
    if len(points) == 2:
        test_detect = True
        start_coordinates = ((points[0][0] + points[1][0]) / 2, (points[0][1] + points[1][1]) / 2)
        if rayon[0] > rayon[1]:  # case bigger cercle backward 
            center2 = points[0] 
            center1 = points[1]  
        else:
            center1 = points[0]
            center2 = points[1]
    else:
        test_detect = False
    if test_detect == False:   #if thymio not detected at the beginning set all the position to 0
        start_coordinates = (0,0)
        center1 = (0,0)
        center2 = (0,0)

    if begin:
        return img, start_coordinates, (center1, center2), test_detect
    else:
        return start_coordinates, (center1, center2), test_detect


def detect_target(image):
    template = cv2.imread('Vision/feuille_rouge.png')
    _, w, h = template.shape[::-1]
    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR', 'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF',
               'cv2.TM_SQDIFF_NORMED']

    img = image.copy()
    # Apply template Matching
    res = cv2.matchTemplate(img, template, cv2.TM_CCORR_NORMED)  
    _, _, min_loc, max_loc = cv2.minMaxLoc(res)  # take min if TM_SQDIFF or TM_SQDIFF_NORMED

    top_left = (max_loc[0], max_loc[1])
    bottom_right = (top_left[0] + w, top_left[1] + h)
    target_coordinates = (top_left[0] + w / 2, top_left[1] + h / 2)
    cv2.rectangle(img, top_left, bottom_right, (255, 255, 255),-1)  # draw white rectangle on the target

    return target_coordinates, img, res


def detect_obstacle(image):  # detect les contours, puis recupere les coins de chaque polygone
   
    img = image.copy()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, gray = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    shapes = []
    shape_center = []
    for i in contours:
        size = cv2.contourArea(i)
        rect = cv2.minAreaRect(i)
        if size < 100000:  #avoid to consider the entire image as a shape
            gray = np.float32(gray)
            mask = np.zeros(gray.shape, dtype="uint8")
            cv2.fillPoly(mask, [i], (255, 255, 255))
            dst = cv2.cornerHarris(mask, 5, 3, 0.04)
            ret, dst = cv2.threshold(dst, 0.1 * dst.max(), 255, 0)
            dst = np.uint8(dst)
            ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            corners = cv2.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)
            if len(corners) > 3:
                shapes.append(corners[1:len(corners)])
                shape_center.append(np.mean(i, axis=0)[0]) # compute the location of the mean of each shape

    return gray, contours, shapes, shape_center


def sort_vertices_clockwise(shapes):  # reorder coin of the shapes clockwise

    lowest = min(shapes, key=lambda x: (x[1], x[0]))
    vertices = sorted(shapes, key=lambda x: math.atan2(x[1] - lowest[1], x[0] - lowest[0]) + 2 * math.pi)
    return vertices


def add_margin(shape, listCenters, margin):  #list center contain the center coordinates of each shape 

    for i in range(len(shape)):
        for j in range(len(shape[i])):
            u = shape[i][j] - listCenters[i]
            u_norm = u / np.linalg.norm(u)
            shape[i][j] = shape[i][j] + margin*u_norm
    return shape
   



def transmit_data(image, show, margin):
    img_start, start_coor, (center1, center2), _ = detect_start1(image)
    target_coor, img_target, _ = detect_target(img_start)
    gray2, _, shapes, shape_center = detect_obstacle(img_target)
   
    for i in range(len(shapes)):
        shapes[i] = sort_vertices_clockwise(shapes[i])
    shapes = add_margin(shapes, shape_center, margin)

    if show:
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, sharey='row')
        ax1.imshow(image)
        ax2.imshow(img_target)
        ax3.imshow(gray2)
        plt.show()

    sz_img = np.shape(image)
    return start_coor, target_coor, shapes, sz_img, (center1, center2)


