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

import pickle

from tensorflow.keras import layers
from tensorflow.keras import models
from tensorflow.keras import optimizers
from tensorflow.keras.utils import plot_model
from tensorflow.keras import backend

import tensorflow as tf
from tensorflow.keras import models
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model

sess1 = tf.compat.v1.Session()    
graph1 = tf.compat.v1.get_default_graph()
set_session(sess1)

class plate_reader:

  def __init__(self):
    # Variables for subscribing and publishing
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
    self.license_pub = rospy.Publisher("/license_plate", String, queue_size=1)
    
    self.model = models.load_model('model.h5')

    self.allchars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
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
    dilate_kernel = np.ones((3,3))
    sharpen_kernel = np.array([[0, -1, 0],
                              [-1, 5, -1],
                              [0, -1, 0]])
    
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
    colour_filter = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(colour_filter, (5,5), 0)
    lower = np.array([0,0,90])
    upper = np.array([0,0,210])
    colour_mask = cv2.inRange(blurred,lower,upper)
    after_mask = cv2.bitwise_and(img_crop, img_crop, mask=colour_mask)
    # reduce_noise = cv2.erode(colour_mask,erode_kernel,iterations=1)
    # reduce_noise = cv2.dilate(reduce_noise,dilate_kernel,iterations=1)
    
    # find contours
    contours, _ = cv2.findContours(colour_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
    areaArray = []
    for _, c in enumerate(contours):
      area = cv2.contourArea(c)
      areaArray.append(area)

    #first sort the array by area
    sortedContours = sorted(zip(areaArray, contours), key=lambda x: x[0], reverse=True)

    # draw the largest contour
    cnt = sortedContours[0][1]
    area = sortedContours[0][0]
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


    # cv2.imshow("image4", blurred)
    # cv2.waitKey(3)
    # cv2.imshow("image3", colour_mask)
    # cv2.waitKey(3)
    cv2.imshow("image2", img_crop)
    cv2.waitKey(3)
    
    # print(area)
    # if there are four corners, and we are close enough (area big enough to avoid noise)
    # do the perspective trannsform
    if area > 6000 and len(sortedPoints) == 4:
      #TODO: This is how the robot realizes it can read a plate
      # print("Grabbing Plate")

      height, width = (500,500)

      # referenced https://arccoder.medium.com/straighten-an-image-of-a-page-using-opencv-313182404b06
      # List the output points in the same order as input
      dstPts = [[0, 0], [width, 0], [0, height], [width, height]]
      # Get the transform
      m = cv2.getPerspectiveTransform(np.float32(sortedPoints), np.float32(dstPts))
      # Transform the image
      norm = cv2.warpPerspective(img_crop, m, (int(width), int(height)+150))

      plate_height = 120
      plate = norm[norm.shape[0]-plate_height:,:]
      # plate = cv2.resize(plate, (500,120), interpolation=cv2.INTER_AREA)
      # plate = cv2.filter2D(plate, ddepth=-1, kernel=sharpen_kernel)

      cv2.imshow("image5", norm)
      cv2.waitKey(3) 
      cv2.imshow("image1", plate)
      cv2.waitKey(3)

      chars = [plate[:,15:115],
                plate[:,115:215],
                plate[:,280:380],
                plate[:,380:480]]
    

      # cv2.imshow("L0", chars[0])
      # cv2.waitKey(3)
      # cv2.imshow("L1", chars[1])
      # cv2.waitKey(3)
      # cv2.imshow("L2", chars[2])
      # cv2.waitKey(3)
      # cv2.imshow("L3", chars[3])
      # cv2.waitKey(3)


      # print(plate.shape)


      # model = models.load_model('saved_model/model')
      # img_aug = np.expand_dims(chars[0],axis=0)
      # predict = model.predict(img_aug)
      # # print(predict)
      # print(self.allchars[np.argmax(predict)])


      plate = ''
      for c in chars:
        img_aug = np.expand_dims(c,axis=0)
        predict = self.model.predict(img_aug)
        plate+=self.allchars[np.argmax(predict)]
      
      print(plate)


    else:
      print("Can't see plate")




    



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