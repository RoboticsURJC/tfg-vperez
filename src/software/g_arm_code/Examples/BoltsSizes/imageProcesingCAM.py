import cv2
import numpy as np
import math, time
import logitechC270_planar


def drawBoundingBox(image, contour):
    # Draws a real bonding box rotated if is neccesary
    rect = cv2.minAreaRect(contour)
    boxContour = cv2.boxPoints(rect)
    boxContour = np.int0(boxContour)
    return cv2.drawContours(image, [boxContour], 0, (0, 255, 0), 11)

def squareContourBoundingBox(contour):
    # Draws a real bonding box rotated if is neccesary
    _, dimensions, _ = cv2.minAreaRect(contour)
        
    ratio = float(max(dimensions)) / min(dimensions)
    
    return ratio < 1.2 # True if is square, false otherwise

def distance(A, B):   
    return math.sqrt( math.pow(A[0]-B[0], 2) + math.pow(A[1]-B[1], 2) ) 

def getBoltSize(ext1, ext2):
    p1x, p1y, _ = logitechC270_planar.get_3d_point_from_2d_point(ext1, 0.3)
    p2x, p2y, _ = logitechC270_planar.get_3d_point_from_2d_point(ext2, 0.3)
    
    return distance( (p1x, p1y), (p2x, p2y) )
    
    
def drawContourBoundingBoxExtremes(img, contour):
    
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    
    if distance(box[0], box[1]) >  distance(box[1], box[2]):
        ext1 = (box[1] + box[2]) / 2
        ext2 = (box[0] + box[3]) / 2
    else:
        ext1 = (box[0] + box[1]) / 2
        ext2 = (box[2] + box[3]) / 2 
    
    ext1 = (int(ext1[0]), int(ext1[1]))
    ext2 = (int(ext2[0]), int(ext2[1]))
    mid = ( int( (ext1[0] + ext2[0]) / 2), int( (ext1[1] + ext2[1]) / 2) )
    
    img = cv2.circle(img, ext1, 35, (0, 255, 0), -1)
    img = cv2.circle(img, ext2, 35, (0, 255, 0), -1)
    
    img = cv2.circle(img, mid, 25, (255, 255, 255), -1)
    
    boltSize = int(getBoltSize(ext1, ext2) * 100)
    text = str(boltSize) + " mm"
    
    cv2.putText(img, text, mid, cv2.FONT_HERSHEY_SIMPLEX, 10, (0, 255, 0), thickness=17)

    
    
    return img
    
def contourCenter(contour):
    moments = cv2.moments(contour)
    cx = int(moments['m10']/moments['m00'])
    cy = int(moments['m01']/moments['m00'])
    
    return (cx, cy)

def contourSize(contour):
    _, dimensions, _ = cv2.minAreaRect(contour)
    return dimensions



'''
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: camera not connected!")
    exit()

startTs = time.time()
runTime = 30 # Secs
'''
while True: #(time.time() - startTs < runTime):
    #frameOk, frame = cap.read() # Lee un fotograma de la cámara
    if True: #frameOk:
        

        #inputImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        inputImage = cv2.imread('imageExample2.jpg', cv2.IMREAD_GRAYSCALE)
        
        # Gaussian Blur with 25x25 kernel
        blur = cv2.GaussianBlur(inputImage, (25, 25), cv2.BORDER_DEFAULT)

        # Make a mask filtering by whiteness
        _, threshold = cv2.threshold(blur, 145, 255, cv2.THRESH_BINARY)

        # Find external contours as a chain of points
        contours, _ = cv2.findContours(
        threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Convert to grayscale the input image to BGR to draw colors on them
        finalContour = cv2.cvtColor(inputImage, cv2.COLOR_GRAY2RGB)
        finalBoundingBox = cv2.cvtColor(inputImage, cv2.COLOR_GRAY2RGB)
        finalShapes = cv2.cvtColor(inputImage, cv2.COLOR_GRAY2RGB)


        for contour in contours:

            if cv2.contourArea(contour) > 1500:

                # Draw bounding box
                finalBoundingBox = drawBoundingBox(finalBoundingBox, contour)

                # Aproximate to a poligon
                poligon = cv2.approxPolyDP(
                    contour, 0.010 * cv2.arcLength(contour, True), True)

                # Draw poligon contour
                finalContour = cv2.drawContours(
                    finalContour, [poligon], 0, (255, 0, 0), 11)

                # Identify shapes
                sides = len(poligon)

                if not squareContourBoundingBox(contour):
                    # Bolt
                    finalShapes = cv2.fillPoly(finalShapes, pts=[contour], color=(255, 0, 255))
                    finalShapes = drawContourBoundingBoxExtremes(finalShapes, contour)
                
            
        # Show progress and result
        stages = np.concatenate((inputImage, blur, threshold), axis=1)
        results = np.concatenate((finalContour, finalBoundingBox, finalShapes), axis=1)


        # Create window with freedom of dimensions
        cv2.namedWindow('processing', cv2.WINDOW_AUTOSIZE)
        # Create window with freedom of dimensions
        cv2.namedWindow('results', cv2.WINDOW_AUTOSIZE)

        stages = cv2.resize(stages, (int(
        stages.shape[1] * 0.1), int(stages.shape[0] * 0.1)), cv2.INTER_AREA)
        results = cv2.resize(results, (int(
        results.shape[1] * 0.1), int(results.shape[0] * 0.1)), cv2.INTER_AREA)

        cv2.imshow('processing', stages)
        cv2.imshow('results', results)


        cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()
