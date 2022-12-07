
import numpy as np
import argparse
import cv2
import time
from matplotlib import pyplot as plt


def detect_start1(img):
   
    points=[]

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    gray = cv2.GaussianBlur(gray,(5,5),0)
    gray = cv2.medianBlur(gray,5)

    gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,3.5)


    #Remove unwanted noise
    kernel = np.ones((3,3),np.uint8)
    gray = cv2.erode(gray,kernel,iterations = 1)
    gray = cv2.dilate(gray,kernel,iterations = 1)       
    

    #circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 10, param1=25, param2=19, minRadius=0, maxRadius=18) #Perform HoughCircle Transform
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=25, param2=19, minRadius=0, maxRadius=20)
   
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")

        for (x, y, r) in circles:

            cv2.circle(img, (x, y), r, (0, 255, 0), 4)
            #cv2.rectangle(img, (x - int(1.4*r), y - int(1.4*r)), (x + int(1.4*r), y + int(1.4*r)), (255, 255, 255), -1)
            pos = (int(x), int(y))
            points.append(pos)
    test_detection = True
    print(points)
    start_coordinates = 0
    if len(points) == 2:
        start_coordinates = ((points[0][0] + points[1][0]) / 2, (points[0][1] + points[1][1]) / 2)
        print("ok")
    else:
        print("revoir hough circles para")

    
    #cv2.imshow('my webcam', img)
    return circles, img, gray, points, start_coordinates

img = cv2.imread("test_2.png")
circles, img , gray, points, start_coordinates= detect_start1(img)
print(circles)
#print(points[0][0])
print(start_coordinates)
plt.imshow(img)
plt.show()