from matplotlib import pyplot as plt
import numpy as np
import cv2
import math
from shapely.geometry import Polygon, Point, LineString

#cap = cv2.VideoCapture(0)


def get_image(cap):
    result, image = cap.read()

    if result:
        # save the img
        cv2.imwrite("premier_test.png", image)
        # print("here")
        return image
    else:
        print("no img read")
        return None


def detect_start(image, show=False, begin = True):
    template1 = cv2.imread('Vision/cerclebleu.png')
    template2 = cv2.imread('Vision/cerclevert.png')
    #template1 = cv2.imread('cerclebleu.png')
    #template2 = cv2.imread('cerclevert.png')
    _, w1, h1 = template1.shape[::-1]
    _, w2, h2 = template2.shape[::-1]
    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR', 'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF',
               'cv2.TM_SQDIFF_NORMED']
    
    img = image.copy()
    # Apply template Matching
    res1 = cv2.matchTemplate(img, template1, cv2.TM_CCORR_NORMED)
    _, _, min_loc1, max_loc1 = cv2.minMaxLoc(res1)  # take min if TM_SQDIFF or TM_SQDIFF_NORMED
    top_left1 = (max_loc1[0], max_loc1[1])
    bottom_right1 = (top_left1[0] + w1, top_left1[1] + h1)
    center1 = (top_left1[0] + w1 / 2, top_left1[1] + h1 / 2)
    if show:
        print("center1")
        print(center1)

    res2 = cv2.matchTemplate(img, template2, cv2.TM_CCORR_NORMED)
    _, _, min_loc2, max_loc2 = cv2.minMaxLoc(res2)  # take min if TM_SQDIFF or TM_SQDIFF_NORMED
    top_left2 = (max_loc2[0], max_loc2[1])
    bottom_right2 = (top_left2[0] + w2, top_left2[1] + h2)
    center2 = (top_left2[0] + w2 / 2, top_left2[1] + h2 / 2)

    if show:
        print("center2")
        print(center2)

    start_coordinates = ((center1[0] + center2[0]) / 2, (center1[1] + center2[1]) / 2)  # pour etre bien centre#
    cv2.rectangle(img, top_left1, bottom_right1, (255, 255, 255),
                  -1)  # rect white, draw rectangle 2 opp corner top left and bottom right
    cv2.rectangle(img, top_left2, bottom_right2, (255, 255, 255),
                  -1)  # rect white, draw rectangle 2 opp corner top left and bottom right

    if begin:
        return start_coordinates, img, res1, res2, center1, center2
    else:
        return (center1, center2)


def detect_target(image):
    template = cv2.imread('Vision/feuille_rouge.png')
    #template = cv2.imread('feuille_rouge.png')
    _, w, h = template.shape[::-1]
    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR', 'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF',
               'cv2.TM_SQDIFF_NORMED']

    img = image.copy()
    # Apply template Matching
    res = cv2.matchTemplate(img, template, cv2.TM_CCORR_NORMED)  # TM_SQDIFF
    _, _, min_loc, max_loc = cv2.minMaxLoc(res)  # take min if TM_SQDIFF or TM_SQDIFF_NORMED

    top_left = (max_loc[0], max_loc[1])
    bottom_right = (top_left[0] + w, top_left[1] + h)
    target_coordinates = (top_left[0] + w / 2, top_left[1] + h / 2)
    cv2.rectangle(img, top_left, bottom_right, (255, 255, 255),
                  -1)  # rect blue, draw rectangle 2 opp corner top left and bottom right

    return target_coordinates, img, res


def detect_obstacle(image):  # detect les contours, puis recupere les coins de chaque polygone
    img = image.copy()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, gray = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    shapes = []
    shape_center = []
    for i in contours:
        size = cv2.contourArea(i)
        rect = cv2.minAreaRect(i)
        if size < 100000:
            gray = np.float32(gray)
            mask = np.zeros(gray.shape, dtype="uint8")
            cv2.fillPoly(mask, [i], (255, 255, 255))
            dst = cv2.cornerHarris(mask, 5, 3, 0.04)
            ret, dst = cv2.threshold(dst, 0.1 * dst.max(), 255, 0)
            dst = np.uint8(dst)
            ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            corners = cv2.cornerSubPix(gray, np.float32(centroids), (5, 5), (-1, -1), criteria)
            # print("corners")
            # print(len(corners))
            if len(corners) > 3:
                shapes.append(corners[1:len(corners)])
                shape_center.append(np.mean(i, axis=0)[0])
                # print('Rombus corners: ')
                # for i in range(1, len(corners)):
                # print(corners[i]

    return gray, contours, shapes


def sort_vertices_clockwise(shapes):  # reoganise les coins dans un ordre sens horaire

    lowest = min(shapes, key=lambda x: (x[1], x[0]))

    vertices = sorted(shapes, key=lambda x: math.atan2(x[1] - lowest[1], x[0] - lowest[0]) + 2 * math.pi)
    # print(vertices)
    return vertices


def add_margin(shapes,
               margin):  # recoit shapes avec les vertices sort clockwise, return les nouveaux points aves la marge
    for i in range(len(shapes)):
        for j in range(len(shapes[i])):
            if j == 0:  # premier element soustrait le dernier a u et le second a v
                u = shapes[i][j] - shapes[i][len(shapes[i]) - 1]
                u_norm = u / np.linalg.norm(u)
                v = shapes[i][j] - shapes[i][j + 1]
                v_norm = v / np.linalg.norm(v)
                shapes[i][j] = shapes[i][j] + margin * u_norm + margin * v_norm
            elif j == (len(shapes[i]) - 1):  # dernier element on soustrait le premier a u et l'avant dernier a v
                u = shapes[i][j] - shapes[i][0]
                v = shapes[i][j] - shapes[i][j - 1]
                u_norm = u / np.linalg.norm(u)
                v_norm = v / np.linalg.norm(v)
                shapes[i][j] = shapes[i][j] + margin * u_norm + margin * v_norm
            else:  # on soustrait le coin avant a u et le coin dapres a v
                u = shapes[i][j] - shapes[i][j - 1]
                v = shapes[i][j] - shapes[i][j + 1]
                u_norm = u / np.linalg.norm(u)
                v_norm = v / np.linalg.norm(v)
                shapes[i][j] = shapes[i][j] + margin * u_norm + margin * v_norm
    return shapes


def secure_path(obstacle, margin):
    """
    Use this function to add a security margin between the obstacle and the path of the robot
    :param obstacle:
    :param margin: the margin by which the polygon is enlarged
    :return: the enlarged obstacles
    """
    obstacle = obstacle.buffer(margin, join_style=2)
    return obstacle


def draw_increase_obstacle(image, poly):  ######################## pas appele pour linstant voir plus tard
    img = image.copy()

    # cree une image identique blanche
    h = len(img)
    w = len(img[0])

    for y in range(h):
        for x in range(w):
            img[y, x] = [255, 255, 255]

    # dessine les polygones
    poly_agr = secure_path(poly, 10)

    test = False  # pour eviter de prendre le premier polygon qui est le contour de limage
    for i in poly_agr:
        if test == True:
            x, y = i.exterior.coords.xy
            points = [[]]
            for k in range(len(x)):
                points.append([int(x[k]), int(y[k])])
            points = points[1:]
            pointss = np.array(points)
            # print(pointss)

            for j in range(len(x) - 1):
                # print(j)
                x[j] = math.floor(x[j])
                y[j] = math.floor(y[j])
                coord1 = (int(x[j]), int(y[j]))
                coord2 = (int(x[j + 1]), int(y[j + 1]))
                # print(type(x[j]))
                # print(y[j])

                # cv2.line(img, coord1, coord2, (0,0,0),1)
            # cv2.drawContours(img, contours, -1, color=(255, 255, 255), thickness=cv2.FILLED)
            # cv2.fillPoly(img, pts=[pointss], color=(0, 0, 0))
        test = True
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    Blur = cv2.GaussianBlur(gray, (5, 5), 1)  # apply blur to roi
    Canny = cv2.Canny(Blur, 10, 50)  # apply canny to roi
    # Canny = cv2.adaptiveThreshold(Blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    # ret, Canny = cv2.threshold(Blur, 50, 255, cv2.THRESH_BINARY)
    # Find my contours
    contours = cv2.findContours(Canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # je draw tout et findcontours ensuite
    # print(contours[0])
    return img, contours


def transmit_data(image, show, margin):
    image = cv2.imread(image)
    start_coor, img_start, res_start1, res_start2, center1, center2 = detect_start(image, show)
    target_coor, img_target, res_target = detect_target(img_start)
    gray2, contours, shapes = detect_obstacle(img_target)
    # print("shape avant")
    # print(shapes)
    for i in range(len(shapes)):
        shapes[i] = sort_vertices_clockwise(shapes[i])

    shapes = add_margin(shapes, margin)

    if show:
        print("shapes")
        # print(shapes[1])
        print("start coord")
        print(start_coor)
        print(target_coor)
        # Converting the original img to black and white

        # Bilateral Filtering
        bilateral = cv2.bilateralFilter(image, 9, 75, 75)
        bw_img = cv2.cvtColor(bilateral, cv2.COLOR_BGR2GRAY)

        # print start goal #########################

        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, sharey='row')
        ax1.imshow(image)
        ax2.imshow(img_target)
        ax3.imshow(gray2)

        plt.title("img, start, target")

        plt.show()

    sz_img = np.shape(image)
    return start_coor, target_coor, shapes, sz_img, (center1,center2)

#img = get_image(cap)
#img = "premier_test.png"
#start_coor, target_coor, shapes, sz_img, (center1,center2) = transmit_data(img,True,10)