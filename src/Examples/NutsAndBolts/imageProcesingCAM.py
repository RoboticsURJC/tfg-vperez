import cv2
import numpy as np
import math, time


def draw_hexagon(im, center, diameter, color):
    radius = int(diameter / 2)  # calculate the radius of the hexagon
    # calculate the apothem of the hexagon
    apothem = int(radius * np.sqrt(3) / 2)
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

def contourCenter(contour):
    moments = cv2.moments(contour)
    cx = int(moments['m10']/moments['m00'])
    cy = int(moments['m01']/moments['m00'])
    
    return (cx, cy)

def contourSize(contour):
    _, dimensions, _ = cv2.minAreaRect(contour)
    return dimensions




cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: camera not connected!")
    exit()

startTs = time.time()
runTime = 30 # Secs

while (time.time() - startTs < runTime):
    frameOk, frame = cap.read() # Lee un fotograma de la cámara
    if frameOk:
        

        inputImage = cv2.imread("imageExample2.jpg", cv2.IMREAD_GRAYSCALE)

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

            if cv2.contourArea(contour) > 10000:

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

                if squareContourBoundingBox(contour):
                    # Nut or washer
                    if sides < 10:
                        finalShapes = draw_hexagon(finalShapes, contourCenter(contour), contourSize(contour)[0], (125, 210, 0))
                    else:
                        finalShapes = cv2.circle(finalShapes, contourCenter(contour), int(contourSize(contour)[0] / 2), (0, 255, 0), -1)
                else:
                    # Bolt
                    finalShapes = cv2.fillPoly(finalShapes, pts=[contour], color=(255, 0, 255))
                
                
            
        # Show progress and result
        stages = np.concatenate((inputImage, blur, threshold), axis=1)
        results = np.concatenate((finalContour, finalBoundingBox, finalShapes), axis=1)


        # Create window with freedom of dimensions
        cv2.namedWindow('processing', cv2.WINDOW_AUTOSIZE)
        # Create window with freedom of dimensions
        cv2.namedWindow('results', cv2.WINDOW_AUTOSIZE)

        stages = cv2.resize(stages, (int(
        stages.shape[1] * 0.10), int(stages.shape[0] * 0.10)), cv2.INTER_AREA)
        results = cv2.resize(results, (int(
        results.shape[1] * 0.10), int(results.shape[0] * 0.10)), cv2.INTER_AREA)

        cv2.imshow('processing', stages)
        cv2.imshow('results', results)


        cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()
