#!/usr/bin/env python3
from __future__ import print_function
from PyQt5 import QtCore, QtGui, QtWidgets
from python_qt_binding import loadUi

import cv2
import sys
import numpy as np

import roslib
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String

import os

class plate_reader:

  def __init__(self):
    # Variables for subscribing and publishing
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
    self.license_pub = rospy.Publisher("/license_plate", String, queue_size=1)
    

    self.startRun = True
    self.testPlate = True

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Constants
    img_height = 400
    erode_kernel = np.ones((3,3))
    dilate_kernel = np.ones((9,9))
    
    # def stop_robot(event):
    #   output = str('Team5,password,-1,ABCD')
    #   try:
    #     self.license_pub.publish(output)
    #     rospy.sleep(0.5)
    #   except CvBridgeError as e:
    #     print(e)

    #   print("Stopping robot")
    #   rospy.signal_shutdown("Timer ended.")

   



    # get image from robot camera
    (rows,cols,channels) = cv_image.shape

    img_crop = cv_image[rows - img_height:,:]
    colour_filter = cv2.cvtColor(img_crop, cv2.COLOR_BGR2LAB)[:,:,0]
    blurred = cv2.GaussianBlur(colour_filter, (5,5), 0)
    colour_mask = cv2.inRange(blurred,97,115)
    reduce_noise = cv2.erode(colour_mask,erode_kernel,iterations=1)
    reduce_noise = cv2.dilate(reduce_noise,dilate_kernel,iterations=1)
    # gray = cv2.filter2D(src=gray,ddepth=-1,kernel=sharpen_kernel)
    
    # find contours
    contours, _ = cv2.findContours(reduce_noise, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
    areaArray = []
    for _, c in enumerate(contours):
      area = cv2.contourArea(c)
      areaArray.append(area)

    #first sort the array by area
    sortedContours = sorted(zip(areaArray, contours), key=lambda x: x[0], reverse=True)

    # draw the largest contour
    cnt = sortedContours[0][1]
    # cv2.drawContours(img_crop,[cnt], -1, (255, 0, 0), 1)

    # referenced: https://stackoverflow.com/questions/41879315/opencv-visualize-polygonal-curves-extracted-with-cv2-approxpolydp
    # define main island contour approx. and hull
    perimeter = cv2.arcLength(cnt,True)
    epsilon = 0.01*perimeter
    approx = cv2.approxPolyDP(cnt,epsilon,True)
    hull = cv2.convexHull(cnt)
    # draw dots at corners
    # cv2.drawContours(img_crop, approx, -1, (0, 0, 255), 3)
    
    # sorts points from descending weighted sum (ie top left, top right, bottom left, bottom right)
    sortedPoints = sorted(approx, key=lambda x: x[0,0]+3*x[0,1])
    # print(approx)
    # print(approx[0,0,0]*approx[0,0,1])
    # print(sortedPoints)

    cv2.imshow("image2", img_crop)
    cv2.waitKey(3)


    height, width = np.divide(cv_image.shape[0:2],5)

    # referenced https://arccoder.medium.com/straighten-an-image-of-a-page-using-opencv-313182404b06
    # List the output points in the same order as input
    dstPts = [[0, 0], [width, 0], [0, height*1.5], [width, height*1.5]]
    # Get the transform
    m = cv2.getPerspectiveTransform(np.float32(sortedPoints), np.float32(dstPts))
    # Transform the image
    norm = cv2.warpPerspective(img_crop, m, (int(width), int(height)+120))

    plate_height = 65
    plate = norm[norm.shape[0]-plate_height:,:]




    

    

    cv2.imshow("image1", plate)
    cv2.waitKey(3)

    plate_num = 1
    plate = 'WXYZ'


    # # start the timer
    # if self.startRun:
    #   output = str('Team5,password,0,ABCD')
    #   self.startRun = False

    #   try:
    #     self.license_pub.publish(output)
    #     rospy.sleep(0.5)
    #   except CvBridgeError as e:
    #     print(e)
        
    #   rospy.Timer(rospy.Duration(10),stop_robot)


def main(args):
  
  rospy.init_node('plate_reader', anonymous=True)
  ic = plate_reader()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)