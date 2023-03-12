import cv2
import numpy as np
import random

img = cv2.imread("imageExample2.jpg", cv2.IMREAD_GRAYSCALE)

def draw_hexagon(im, center, diameter, color):
    radius = int(diameter / 2)  # calculate the radius of the hexagon
    apothem = int(radius * np.sqrt(3) / 2)  # calculate the apothem of the hexagon
    x, y = center  # get the coordinates of the center of the hexagon

    # Calculate the coordinates of the vertices of the hexagon
    vertices = np.array([
        [x, y - radius],
        [x + apothem, y - radius/2],
        [x + apothem, y + radius/2],
        [x, y + radius],
        [x - apothem, y + radius/2],
        [x - apothem, y - radius/2]
    ], dtype=np.int32)

    # Draw the filled hexagon on the image
    return cv2.fillPoly(im, [vertices], color)


def process1(img):
    
    # blur - threshold - dilate - erode
    
    smooth = cv2.GaussianBlur(img,(25,25),cv2.BORDER_DEFAULT)
    
    _, threshold = cv2.threshold(smooth, 145, 255, cv2.THRESH_BINARY)
    
 
    # Taking a matrix of size 5 as the kernel
    kernel = np.ones((5, 5), np.uint8)


    #dilate = cv2.dilate(threshold, kernel, iterations=3)

    
    contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    

        
    finalContour = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    
    finalBoundingBox = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    
    finalShapes = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    
    
    for i in range(len(contours)):
        
        if cv2.contourArea(contours[i]) < 10000:
            continue
        
        poligon = cv2.approxPolyDP(contours[i], 0.010* cv2.arcLength(contours[i], True), True)
                
        color = (255, 0, 0)#(random.randint(0,255), random.randint(0,255), random.randint(0,255))
        finalContour = cv2.drawContours(finalContour, [poligon], -1, color, 11)
        
        (x, y, w, h) = cv2.boundingRect(contours[i])
        finalBoundingBox = cv2.rectangle(finalBoundingBox, (x,y), (x+w, y+h), (0, 255, 0), 10)
        
        
        # Identification
        sides = len(poligon)
        
        aspectRatio = float(w) / h
        
        print("Sides: " + str(sides))
        print("Ratio: " + str(aspectRatio))
        
        center = (int(x+w/2), int(y+h/2))
        
        if sides < 10 and aspectRatio > 0.8 and aspectRatio < 1.2:
            finalShapes = draw_hexagon(finalShapes, center, w, (0, 255, 255))

        if aspectRatio < 0.8 or aspectRatio > 1.2:
            finalShapes = cv2.fillPoly(finalShapes, [ np.array( contours[i] ) ], (255, 0, 255))
        
        if sides > 10 and aspectRatio > 0.8 and aspectRatio < 1.2:
            finalShapes = cv2.circle(finalShapes, center, int(w/2), color, -1)
        
    
    stages = np.concatenate((img, threshold), axis=1)
    
    results = np.concatenate((finalContour, finalBoundingBox, finalShapes), axis=1)  
    cv2.namedWindow('processing', cv2.WINDOW_AUTOSIZE)    # Create window with freedom of dimensions    
    cv2.namedWindow('results', cv2.WINDOW_AUTOSIZE)    # Create window with freedom of dimensions
    
    stages = cv2.resize(stages, (int(stages.shape[1] * 0.10), int(stages.shape[0] * 0.10)), cv2.INTER_AREA)
    
    results = cv2.resize(results, (int(results.shape[1] * 0.10), int(results.shape[0] * 0.10)), cv2.INTER_AREA)
 
    cv2.imshow('processing', stages)
    cv2.imshow('results', results)
    
    





process1(img)


cv2.waitKey(10000)
cv2.destroyAllWindows()