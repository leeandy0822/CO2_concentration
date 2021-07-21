# import the necessary packages
from imutils.perspective import four_point_transform
from imutils import contours
import imutils
import cv2
import numpy as np

class image_process:
    def __init__(self,image):
        self.image = image
        
    def preprocessing(self):
        image = imutils.resize(self.image, height=500)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


        # cut the region
        x = 230
        y = 435
        w = 140
        h = 160
        
        warped = image[y:y+h, x:x+w]
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

         # threshold the warped image, then apply a series of morphological operations to cleanup the thresholded image
        thresh = cv2.threshold(gray, 10, 200,
	         cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4,1))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # erostion and dilation can help us filter the useless info and strengthen the number info
        kernel_e = np.ones((4,4), np.uint8)
        erosion = cv2.erode(thresh, kernel, iterations = 5)
        kernel_d = np.ones((4,4), np.uint8)
        dilation = cv2.dilate(erosion, kernel, iterations = 5)
  
        # find contours in the thresholded image, then initialize the
        # digit contours lists
        cnts = cv2.findContours(dilation.copy(), cv2.RETR_EXTERNAL,
	        cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        
          
        digitCnts = []

         # loop over the digit area candidates
        for c in cnts:
	        # compute the bounding box of the contour
	        (x, y, w, h) = cv2.boundingRect(c)
	        # if the contour is sufficiently large, it must be a digit
	        if (w >= 10 and w<= 140 ) and (h >= 10 and h<= 140):
	  	        digitCnts.append(c)
            
        return cnts, warped,image,thresh,digitCnts
    
    

    
  