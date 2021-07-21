#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
from inspect import getcoroutinelocals
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Float32
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
# import the necessary packages
from imutils.perspective import four_point_transform
from imutils import contours
import imutils
import cv2
import numpy as np
import time
# define the dictionary of digit segments so we can identify
# each digit on the thermostat
DIGITS_LOOKUP = {
	(1, 1, 1, 0, 1, 1, 1): 0,
	(0, 0, 1, 0, 0, 1, 0): 1,
	(1, 0, 1, 1, 1, 1, 0): 2,
	(1, 0, 1, 1, 0, 1, 1): 3,
	(0, 1, 1, 1, 0, 1, 0): 4,
	(1, 1, 0, 1, 0, 1, 1): 5,
	(1, 1, 0, 1, 1, 1, 1): 6,
	(1, 0, 1, 0, 0, 1, 0): 7,
	(1, 1, 1, 1, 1, 1, 1): 8,
	(1, 1, 1, 1, 0, 1, 1): 9
}



def callback(data):
  result = 0
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  image = br.imgmsg_to_cv2(data)
  # Display image
  # cv2.imshow("camera", current_frame)
   
  # cv2.waitKey(1)


  # load the example image
  # image = cv2.imread("example.jpg")
  # pre-process the image by resizing it, converting it to
  # graycale, blurring it, and computing an edge map
  image = imutils.resize(image, height=500)
  image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

  #裁切區域的 x 與 y 座標（左上角）
  x = 230
  y = 435

  # 裁切區域的長度與寬度
  w = 140
  h = 160
  # 裁切圖片
  warped = image[y:y+h, x:x+w]
  gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)



  # threshold the warped image, then apply a series of morphological
  # operations to cleanup the thresholded image
  thresh = cv2.threshold(gray, 10, 200,
	  cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]

  kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,1))

  thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
  # display the digits

  kernel_e = np.ones((4,4), np.uint8)
  erosion = cv2.erode(thresh, kernel, iterations = 5)
  
  kernel_d = np.ones((5,5), np.uint8)
  dilation = cv2.dilate(erosion, kernel, iterations = 4)
  
  # find contours in the thresholded image, then initialize the
  # digit contours lists
  cnts = cv2.findContours(dilation.copy(), cv2.RETR_EXTERNAL,
	  cv2.CHAIN_APPROX_SIMPLE)

  
  cnts = imutils.grab_contours(cnts)
  digitCnts = []
  print(len(cnts))
  # loop over the digit area candidates
  for c in cnts:
	  # compute the bounding box of the contour
	  (x, y, w, h) = cv2.boundingRect(c)
	  # if the contour is sufficiently large, it must be a digit
	  if (w >= 20 and w<=90) and (h >= 40 ):
	  	digitCnts.append(c)

  print(len(digitCnts))
  # sort the contours from left-to-right, then initialize the
  # actual digits themselves
  try:
    digitCnts = contours.sort_contours(digitCnts,
  	  method="left-to-right")[0]
    global digits
    digits = []

    draw_img0 = cv2.drawContours(warped.copy(),digitCnts,-1,(0,255,255),3)
    cv2.imshow("draw",draw_img0)

  except:
    print("no available digits")

  try:
      # loop over each of the digits
    for c in digitCnts:
	    # extract the digit ROI
	    (x, y, w, h) = cv2.boundingRect(c)
	    roi = thresh[y:y + h, x:x + w]
	    # compute the width and height of each of the 7 segments
	    # we are going to examine
	    (roiH, roiW) = roi.shape
	    (dW, dH) = (int(roiW * 0.3), int(roiH * 0.2))
	    dHC = int(roiH * 0.05)
	    # define the set of 7 segments
	    segments = [
		    ((0, 0), (w, dH)),	# top
		   ((5, 0), (dW+2, h // 2)),	# top-left
		   ((w - dW, 0), (w, h // 2)),	# top-right
		    ((0, (h // 2) - dHC) , (w, (h // 2) + dHC)), # center
		   ((0, h // 2), (dW, h)),	# bottom-left
		    ((w - dW-3, h // 2), (w-5, h)),	# bottom-right
		   ((0, h - dH), (w-5, h))	# bottom
	    ]
	    on = [0] * len(segments)
 
 
 	    # loop over the segments
	    for (i, ((xA, yA), (xB, yB))) in enumerate(segments):
	  	  # extract the segment ROI, count the total number of
	  	  # thresholded pixels in the segment, and then compute
	  	  # the area of the segment
		    segROI = roi[yA:yB, xA:xB]
		    total = cv2.countNonZero(segROI)
		    area = (xB - xA) * (yB - yA)
		    # if the total number of non-zero pixels is greater than
		    # 50% of the area, mark the segment as "on"
		    # print(total/float(area))
		    if total / float(area) > 0.5:
		  	  on[i]= 1
       
       
          
	      # lookup the digit and draw it on the image

	    digit = DIGITS_LOOKUP[tuple(on)]
	    digits.append(digit)
	    cv2.rectangle(warped, (x, y), (x + w, y + h), (0, 255, 0), 1)
	    cv2.putText(warped, str(digit), (x - 10, y - 10),
		    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

  except:
    print("Number detection error")

  

  # display the digits
  try:
    print(u"{}.{}{} \u00b0C".format(*digits))
    print(digits)
    result = digits[0]+digits[1]*0.1+digits[2]*0.01


  except:
    print("no number detected")  
    
  cv2.imshow("output",warped)
  cv2.imshow("thresh",dilation)
  cv2.imshow("screen",image)
  
  print("result: ",result)
  img_msg = br.cv2_to_imgmsg(image, encoding="passthrough")
  
  radiation_pub.publish(result)
  img_pub.publish(img_msg)
  
  cv2.waitKey(1)

def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)

   
  # Node is subscribing to the video_frames topic
  try:
    rospy.Subscriber('/camera/color/image_rect_color', Image, callback)
  except:
    print("subscribe error")
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  global img_pub
  global radiation_pub
  img_pub = rospy.Publisher('radiation_img_detect', Image, queue_size=30)
  radiation_pub = rospy.Publisher('radiation_meter', Float32, queue_size=10)
  
  receive_message()
