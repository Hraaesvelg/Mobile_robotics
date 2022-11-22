import matplotlib.pyplot as plt
import cv2
import numpy as np




def get_image():
    cap = cv2.VideoCapture(0)
    result,image = cap.read()

    if result:
        # save the image
        cv2.imwrite("test1.png", image)
        print("here")
    else:
        print("no image read")
    return image



def detect_start(image):

    template = cv2.imread('feuille_bleue.png')
    _, w, h  = template.shape[::-1]
    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR','cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
    
    img = image.copy()
    # Apply template Matching
    res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF) 
    _, _, min_loc, max_loc = cv2.minMaxLoc(res)   #take min if TM_SQDIFF or TM_SQDIFF_NORMED

    top_left = (max_loc[0], max_loc[1])
    bottom_right = (top_left[0] + w, top_left[1] + h)
    start_coordinates = (top_left[0] + w/2, top_left[1] + h/2)  #pour etre bien centre#
    cv2.rectangle(img,top_left, bottom_right, (0, 0, 255), -1)  #rect blue, draw rectangle 2 opp corner top left and bottom right
    
    
    return start_coordinates,img,res


def detect_target(image):
    template = cv2.imread('feuille_rouge.png')
    _, w, h  = template.shape[::-1]
    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR','cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
    
    img = image.copy()
    # Apply template Matching
    res = cv2.matchTemplate(img,template,3)  #TM_SQDIFF
    _, _, min_loc, max_loc = cv2.minMaxLoc(res)   #take min if TM_SQDIFF or TM_SQDIFF_NORMED

    top_left = (max_loc[0], max_loc[1])
    bottom_right = (top_left[0] + w, top_left[1] + h)
    target_coordinates = (top_left[0] + w/2, top_left[1] + h/2)
    cv2.rectangle(img,top_left, bottom_right, (255, 0, 0), -1)  #rect blue, draw rectangle 2 opp corner top left and bottom right

    return target_coordinates,img,res

def detect_obstacle(image):   # pas necessaire coord des obstacles, juste une image et on fait path planning dessur
    obstacle = 0
    return obstacle







#main  
print("reading")
#image = get_image()
image = cv2.imread("test.png")
start_coor, img_start, res_start = detect_start(image)
target_coor, img_target, res_target = detect_target(image)

print(start_coor)
print(target_coor)


# Converting the original image to black and white

# Bilateral Filtering 
bilateral = cv2.bilateralFilter(image,9,75,75)

bw_img = cv2.cvtColor(bilateral, cv2.COLOR_BGR2GRAY)



plt.figure(1)
th3 = cv2.adaptiveThreshold(bw_img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,101,4)
plt.title("threshold gauss")
plt.imshow(th3)


plt.figure(2)
plt.title("bilateral + color")
plt.imshow(bw_img)

plt.figure(3)
th2 = cv2.adaptiveThreshold(bw_img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)   #le mieux
plt.title("threshold mean")
plt.imshow(th2)

plt.figure(4)
ret,th1 = cv2.threshold(bw_img,165, 220,cv2.THRESH_BINARY)
plt.title("thresh binary")
plt.imshow(th1)
#print start goal #########################

fig, (ax1, ax2, ax3)  = plt.subplots(1, 3,sharey='row')
ax1.imshow(image)
ax2.imshow(img_start)
ax3.imshow(img_target)

plt.title("image, start, target")
plt.show()

print("end")


