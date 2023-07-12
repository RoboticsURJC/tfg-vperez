import cv2
import imageProcessing
import logitechC270_planar
import math

def getBoltSize(boltExtremes):
    pixel_1, pixel_2 = boltExtremes
    
    point_1 = logitechC270_planar.get_3d_point_from_2d_point(pixel_1, 0.3)
    point_2 = logitechC270_planar.get_3d_point_from_2d_point(pixel_2, 0.3)
    
    return math.sqrt(math.pow(point_1[0]-point_2[0], 2) + math.pow(point_1[1]-point_2[1], 2) )

inputImage = cv2.imread('imageExample2.jpg')

imageProcessing.SHOW = True
mask = imageProcessing.filterImage(inputImage)

contours = imageProcessing.getBoltContours(mask)


preview = inputImage.copy()

for contour in contours:
    
    boltExtremes = imageProcessing.getContourExtremes(contour)
    boltSize = getBoltSize(boltExtremes)
    
    mid = ( int((boltExtremes[0][0] + boltExtremes[1][0]) / 2), int((boltExtremes[0][1] + boltExtremes[1][1]) / 2))
    
    preview = cv2.circle(preview, boltExtremes[0], 35, (255, 0, 0), -1)
    preview = cv2.circle(preview, boltExtremes[1], 35, (255, 0, 0), -1)
      
    # Contour gravity center
    M = cv2.moments(contour)
    
    mid = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
    
    preview = cv2.circle(preview, mid, 35, (0, 0, 0), -1)
    
    text = "   " + str(int(boltSize * 100)) + " mm"
    preview = cv2.putText(preview, text, mid, cv2.FONT_HERSHEY_SIMPLEX, 10, (0, 255, 0), thickness=40)
    

preview = cv2.resize(preview, (int(preview.shape[1] * 0.1), int(preview.shape[0] * 0.1)), cv2.INTER_AREA)   

cv2.namedWindow('Preview', cv2.WINDOW_AUTOSIZE)
cv2.imshow('Preview', preview)

cv2.waitKey(10000)


