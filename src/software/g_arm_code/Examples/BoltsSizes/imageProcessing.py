import cv2, math

SHOW = False

def __squareShape(contour):
    # Draws a real bonding box rotated if is neccesary
    _, dimensions, _ = cv2.minAreaRect(contour)     
    ratio = float(max(dimensions)) / min(dimensions)  
    return ratio < 1.2 # True if is square, false otherwise

def __distance(A, B):   
    return math.sqrt( math.pow(A[0]-B[0], 2) + math.pow(A[1]-B[1], 2) ) 

def filterImage(inputImage):
    img = cv2.cvtColor(inputImage, cv2.COLOR_BGR2GRAY)
        
    # Gaussian Blur with 25x25 kernel
    blur = cv2.GaussianBlur(img, (25, 25), cv2.BORDER_DEFAULT)

    # Make a mask filtering by whiteness
    _, threshold = cv2.threshold(blur, 145, 255, cv2.THRESH_BINARY)
     
    if SHOW: # DEBUG
        cv2.namedWindow('Filter', cv2.WINDOW_AUTOSIZE)
        show = cv2.resize(threshold, (int(threshold.shape[1] * 0.1), int(threshold.shape[0] * 0.1)), cv2.INTER_AREA)
        cv2.imshow('Filter', show)
    
    return threshold

def getBoltContours(img):
    
    boltContours = []
    
    # Find external contours as a chain of points
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        if cv2.contourArea(contour) > 1500 and not __squareShape(contour):
            boltContours.append(contour)
    
    if SHOW: # DEBUG
        contourImg = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        show = cv2.drawContours(contourImg, boltContours, -1, (0, 255, 0), 20)
        
        show = cv2.resize(contourImg, (int(contourImg.shape[1] * 0.1), int(contourImg.shape[0] * 0.1)), cv2.INTER_AREA)   
        cv2.namedWindow('Contour', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Contour', show)

    return boltContours

def getContourExtremes(contour):
    rect = cv2.minAreaRect(contour)
    box = cv2.boxPoints(rect)
    
    if __distance(box[0], box[1]) >  __distance(box[1], box[2]):
        ext1 = (box[1] + box[2]) / 2
        ext2 = (box[0] + box[3]) / 2
    else:
        ext1 = (box[0] + box[1]) / 2
        ext2 = (box[2] + box[3]) / 2 
    
    ext1 = (int(ext1[0]), int(ext1[1]))
    ext2 = (int(ext2[0]), int(ext2[1]))
    
    return (ext1, ext2)