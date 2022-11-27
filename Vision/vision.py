from matplotlib import pyplot as plt
import numpy as np
import cv2


def get_image():
    cap = cv2.VideoCapture(0)
    result, image = cap.read()

    if result:
        # save the img
        cv2.imwrite("cercle.png", image)
        print("here")
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
    d = 1  # ksize seulement impair
    sigmacolo = 10
    sigmaspace = 12
    gray2 = cv2.bilateralFilter(gray1, d, sigmacolo, sigmaspace)

    gray3 = cv2.adaptiveThreshold(gray2, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    contours, hierarchy = cv2.findContours(gray3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return gray3, contours, hierarchy


def transmit_data(image, show):
    image = cv2.imread(image)
    start_coor, img_start, res_start1, res_start2, center1, center2 = detect_start(image, show)
    target_coor, img_target, res_target = detect_target(img_start)
    gray2, contours, hierarchy = detect_obstacle(img_target)

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
