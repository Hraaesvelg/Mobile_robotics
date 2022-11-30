from matplotlib import pyplot as plt
import numpy as np
import cv2
import math
from shapely.geometry import Polygon, Point, LineString

cap = cv2.VideoCapture(0)
def get_image(cap):
   
    result, image = cap.read()

    if result:
        # save the img
        cv2.imwrite("image_simple.png", image)
        #print("here")
    else:
        print("no img read")

    return image


def detect_start(image, show):
    template1 = cv2.imread('Vision/cerclebleu.png')
    template2 = cv2.imread('Vision/cerclevert.png')
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

    return start_coordinates, img, res1, res2, center1, center2


def detect_target(image):
    template = cv2.imread('Vision/feuille_rouge.png')
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


def detect_obstacle(image):  # pas necessaire coord des obstacles, juste une img et on fait path planning dessur
    img = image.copy()
    filtered_img = cv2.bilateralFilter(img, 3, 75, 75)
    gray = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
    ret, gray1 = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
    # d = 1  # ksize seulement impair
    # sigmacolo = 10
    # sigmaspace = 12
    # gray2 = cv2.bilateralFilter(gray1, d, sigmacolo, sigmaspace)

    gray3 = cv2.adaptiveThreshold(gray1, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    contours, hierarchy = cv2.findContours(gray3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return gray3, contours, hierarchy


def secure_path(obstacle, margin):
    """
    Use this function to add a security margin between the obstacle and the path of the robot
    :param obstacle:
    :param margin: the margin by which the polygon is enlarged
    :return: the enlarged obstacles
    """
    obstacle=obstacle.buffer(margin, join_style=2)
    return obstacle

def draw_increase_obstacle(image,poly):
    img = image.copy()

    #cree une image identique blanche
    h = len(img)
    w = len(img[0])

    for y in range(h):
        for x in range(w):
            img[y,x] = [255,255,255]

    #dessine les polygones
    poly_agr =secure_path(poly, 30)

    print(poly_agr)
    test  = False  # pour eviter de prendre le premier polygon qui est le contour de limage
    for i in poly_agr:
        if test == True:
            x, y = i.exterior.coords.xy
            points = [[]]
            for k in range(len(x)):
                points.append([int(x[k]),int(y[k])])
            points = points[1:]
            pointss = np.array(points)
            #print(pointss)
            
            for j in range(len(x)-1):
                #print(j)
                x[j] = math.floor(x[j])
                y[j] = math.floor(y[j])
                coord1 = (int(x[j]), int(y[j]))
                coord2 = (int(x[j+1]),int(y[j+1]))
                #print(type(x[j]))
                #print(y[j])
                
                cv2.line(img, coord1, coord2, (0,0,0),1)
            #cv2.drawContours(img, contours, -1, color=(255, 255, 255), thickness=cv2.FILLED)
            #cv2.fillPoly(img, pts=[pointss], color=(0, 0, 0))
        test = True
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    Blur=cv2.GaussianBlur(gray,(5,5),1) #apply blur to roi
    Canny=cv2.Canny(Blur,10,50) #apply canny to roi
    #Canny = cv2.adaptiveThreshold(Blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    #ret, Canny = cv2.threshold(Blur, 50, 255, cv2.THRESH_BINARY)
    #Find my contours
    contours =cv2.findContours(Canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    #contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #je draw tout et findcontours ensuite
    print(contours[0])
    return img,contours

def transmit_data(image, show):
    image = cv2.imread(image)
    start_coor, img_start, res_start1, res_start2, center1, center2 = detect_start(image, show)
    target_coor, img_target, res_target = detect_target(img_start)
    gray2, contours, hierarchy = detect_obstacle(img_target)

    #contours = secure_path(contours,30) # agrandissement des obstacles sur l'image

    show = True
    if show:
        print("contours")
        print(contours)
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
    return (center1, center2), target_coor, contours, sz_img
