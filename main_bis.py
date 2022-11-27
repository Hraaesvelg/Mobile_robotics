import matplotlib.pyplot as plt
import cv2
import numpy as np




def get_image():
    cap = cv2.VideoCapture(0)
    result,image = cap.read()

    if result:
        # save the image
        cv2.imwrite("field.png", image)
        print("here")
    else:
        print("no image read")
    
    return image



def detect_start(image):

    template1 = cv2.imread('cercleb.png')
    template2 = cv2.imread('cerclev.png')
    _, w1, h1  = template1.shape[::-1]
    _, w2, h2  = template2.shape[::-1]
    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR','cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
    
    img = image.copy()
    # Apply template Matching
    res1 = cv2.matchTemplate(img,template1,cv2.TM_CCORR_NORMED) 
    _, _, min_loc1, max_loc1 = cv2.minMaxLoc(res1)   #take min if TM_SQDIFF or TM_SQDIFF_NORMED
    top_left1 = (max_loc1[0], max_loc1[1])
    bottom_right1 = (top_left1[0] + w1, top_left1[1] + h1)
    center1 = (top_left1[0] + w1/2, top_left1[1] + h1/2)
    print("center1")
    print(center1)
   
    
    

    res2 = cv2.matchTemplate(img,template2,cv2.TM_CCORR_NORMED) 
    _, _, min_loc2, max_loc2 = cv2.minMaxLoc(res2)   #take min if TM_SQDIFF or TM_SQDIFF_NORMED
    top_left2 = (max_loc2[0], max_loc2[1])
    bottom_right2 = (top_left2[0] + w2, top_left2[1] + h2)
    center2 = (top_left2[0] + w2/2, top_left2[1] + h2/2)
    print("center2")
    print(center2)


    start_coordinates = ((center1[0] + center2[0])/2, (center1[1] + center2[1])/2 )  #pour etre bien centre#
    cv2.rectangle(img,top_left1, bottom_right1, (255, 255, 255), -1)  #rect white, draw rectangle 2 opp corner top left and bottom right
    cv2.rectangle(img,top_left2, bottom_right2, (255, 255, 255), -1)  #rect white, draw rectangle 2 opp corner top left and bottom right
    
    
    return start_coordinates,img,res1,res2


def detect_target(image):
    template = cv2.imread('feuille_rouge.png')
    _, w, h  = template.shape[::-1]
    methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR','cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
    
    img = image.copy()
    # Apply template Matching
    res = cv2.matchTemplate(img,template,cv2.TM_CCORR_NORMED)  #TM_SQDIFF
    _, _, min_loc, max_loc = cv2.minMaxLoc(res)   #take min if TM_SQDIFF or TM_SQDIFF_NORMED

    top_left = (max_loc[0], max_loc[1])
    bottom_right = (top_left[0] + w, top_left[1] + h)
    target_coordinates = (top_left[0] + w/2, top_left[1] + h/2)
    cv2.rectangle(img,top_left, bottom_right, (255, 255, 255), -1)  #rect blue, draw rectangle 2 opp corner top left and bottom right

    return target_coordinates,img,res

def detect_obstacle(image):   # pas necessaire coord des obstacles, juste une image et on fait path planning dessur
    img = image.copy()
    filtered_img = cv2.bilateralFilter(img,3,75,75)
    gray = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
    #ret, gray1 = cv2.threshold(gray,50,255,cv2.THRESH_BINARY)
    ret, gray1 = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)
    # d = 1 #ksize seulement impair
    # sigmacolo = 10
    # sigmaspace = 12
    # gray2 =  cv2.bilateralFilter(gray1,d,sigmacolo,sigmaspace)
    
    #gray3 = cv2.adaptiveThreshold(gray1,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)
    cv2.imwrite("contour.png", gray1)
    contours, hierarchy = cv2.findContours(gray1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
   

    return gray1,contours, hierarchy
    






#main  
print("reading")
#image = get_image()
print("end reading")
# print("reading1")
# image = get_image()
# print("end reading1")

image = cv2.imread("field.png")
start_coor, img_start, res_start1,res_start2 = detect_start(image)
target_coor, img_target, res_target = detect_target(img_start)
gray2, contours, hierarchy = detect_obstacle(img_target)

print("contours")
print(contours)

print("start coord")
print(start_coor)
print(target_coor)


# Converting the original image to black and white

# Bilateral Filtering 
bilateral = cv2.bilateralFilter(image,9,75,75)

bw_img = cv2.cvtColor(bilateral, cv2.COLOR_BGR2GRAY)



#plt.figure(1)
# th3 = cv2.adaptiveThreshold(bw_img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,101,4)
# plt.title("threshold gauss")
#plt.imshow(contours)


# plt.figure(2)
# plt.title("bilateral + color")
# plt.imshow(bw_img)

# plt.figure(3)
# th2 = cv2.adaptiveThreshold(bw_img,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,2)   #le mieux
# plt.title("threshold mean")
# plt.imshow(th2)

# plt.figure(4)
# ret,th1 = cv2.threshold(bw_img,165, 220,cv2.THRESH_BINARY)
# plt.title("thresh binary")
# plt.imshow(th1)
#print start goal #########################

fig, (ax1, ax2, ax3)  = plt.subplots(1, 3,sharey='row')
ax1.imshow(image)
ax2.imshow(img_target)
ax3.imshow(gray2)




plt.title("image, start, target")
plt.show()

print("end")


