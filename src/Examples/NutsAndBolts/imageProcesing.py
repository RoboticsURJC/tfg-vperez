import cv2
import numpy as np
import random

img = cv2.imread("imageExample.jpg", cv2.IMREAD_GRAYSCALE)

def process1(img):
    
    # blur - threshold - dilate - erode
    
    smooth = cv2.GaussianBlur(img,(25,25),cv2.BORDER_DEFAULT)
    
    _, threshold = cv2.threshold(smooth, 145, 255, cv2.THRESH_BINARY)
    
 
    # Taking a matrix of size 5 as the kernel
    kernel = np.ones((5, 5), np.uint8)


    #dilate = cv2.dilate(threshold, kernel, iterations=3)

    
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    #for cnt in contours:
    #   (x, y, w, h) = cv2.boundingRect(cnt)
    final = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    
    
    for i in range(len(contours)):
        
        if cv2.contourArea(contours[i]) < 10000:
            continue
        
        poligon = cv2.approxPolyDP(contours[i], 0.010* cv2.arcLength(contours[i], True), True)
        
        print(len(poligon))
        
        color = (255, 0, 0)#(random.randint(0,255), random.randint(0,255), random.randint(0,255))
        final = cv2.drawContours(final, [poligon], -1, color, 11)
    
    stages = np.concatenate((img, threshold), axis=1)
    
    cv2.namedWindow('p1', cv2.WINDOW_AUTOSIZE)    # Create window with freedom of dimensions    
    
    stages = cv2.resize(stages, (int(stages.shape[1] * 0.15), int(stages.shape[0] * 0.15)), cv2.INTER_AREA)
    
    final = cv2.resize(final, (int(final.shape[1] * 0.25), int(final.shape[0] * 0.25)), cv2.INTER_AREA)
    
    cv2.imshow('p1', stages)
    cv2.imshow('p2', final)





process1(img)


cv2.waitKey(10000)
cv2.destroyAllWindows()