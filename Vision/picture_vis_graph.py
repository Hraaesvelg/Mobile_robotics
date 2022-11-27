import matplotlib.pyplot as plt
import cv2
import numpy as np





image = cv2.imread("noir.png")
copy = image.copy()
# Find Contours of each shapes
gray = cv2.cvtColor(copy, cv2.COLOR_BGR2GRAY)
th2 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
th3 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
#ret,gray = cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
#gray = cv2.erode(gray, (2,2), iterations = 1)
#gray = cv2.blur(gray,(5,5))
th2 = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
#ret,gray = cv2.threshold(gray,60,255,cv2.THRESH_BINARY_INV)
#contours, hierarchy = cv2.findContours(gray,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

fig, (ax1, ax2,ax3)  = plt.subplots(1, 3,sharey='row')
ax1.imshow(gray)
ax2.imshow(th2)
ax3.imshow(th3)

plt.title("image, start, target")
plt.show()