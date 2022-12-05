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

from random import randint

sess1 = tf.compat.v1.Session()    
graph1 = tf.compat.v1.get_default_graph()
set_session(sess1)

class plate_reader:

  def __init__(self):
    
    ##################### UNCOMMENT FOR INDIV. TESTING #####################
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.read_plate)
    self.license_pub = rospy.Publisher("/license_plate", String, queue_size=1)
    ##################### UNCOMMENT FOR INDIV. TESTING #####################
    
    self.model = models.load_model('plate_models/new_model.h5')
    self.allchars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
    # self.startRun = True
    # self.testPlate = True

  def read_plate(self,cv_image):
    
    ##################### UNCOMMENT FOR INDIV. TESTING #####################
    try:
      cv_image = self.bridge.imgmsg_to_cv2(cv_image,"bgr8")
    except CvBridgeError as e:
      print(e)
    ##################### UNCOMMENT FOR INDIV. TESTING #####################



    # Constants
    img_height = 400
    sharpen_kernel = np.array([[0, -1, 0],
                              [-1, 5, -1],
                              [0, -1, 0]])
    erode_kernel3 = np.ones((3,3))
    dilate_kernel3 = np.ones((3,3))
    
    
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
    hsv = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
    
    # cv2.imshow("image3", hsv)
    # cv2.waitKey(3)

    blurred = cv2.GaussianBlur(hsv, (5,5), 0)
    lower = np.array([80,0,75])
    upper = np.array([150,65,180])
    colour_mask = cv2.inRange(blurred,lower,upper)
    # after_mask = cv2.bitwise_and(img_crop, img_crop, mask=colour_mask)
    # reduce_noise = cv2.erode(colour_mask,erode_kernel,iterations=1)
    # reduce_noise = cv2.dilate(reduce_noise,dilate_kernel,iterations=1)
    
    # find contours
    contours, _ = cv2.findContours(colour_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 



    # def find_corners(contour):
    #   # input:    contours of a shape
    #   # returns:  sorted list of the corners of a rectangle:
    #   #           (top left, top right, bottom left, bottom right)






    if len(contours) > 0:
      #first sort the array by area
      sortedContours = sorted(contours, key=cv2.contourArea, reverse=True)

      # draw the largest contour
      cnt = sortedContours[0]
      img_draw = np.copy(img_crop)
      cv2.drawContours(img_draw,[cnt], -1, (255, 0, 0), 3)
      check_area = cv2.contourArea(cnt)




      # referenced: https://stackoverflow.com/questions/41879315/opencv-visualize-polygonal-curves-extracted-with-cv2-approxpolydp
      # define main island contour approx. and hull
      perimeter = cv2.arcLength(cnt,True)
      epsilon = 0.01*perimeter
      approx = cv2.approxPolyDP(cnt,epsilon,True)
      hull = cv2.convexHull(cnt)
      # draw dots at corners
      cv2.drawContours(img_draw, approx, -1, (0, 0, 255), 3)
      sortedPoints = sorted(approx, key=lambda x: 3*x[0,0]+x[0,1])
      # print(approx)
      # print(approx[0,0,0]*approx[0,0,1])
      # print(sortedPoints)






      # if there are four corners do the perspective trannsform
      if len(sortedPoints) == 4 and check_area > 2000:
        img_text = cv2.putText(img_draw, str(cv2.contourArea(cnt))+"\nI see a plate", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        # print("Grabbing Plate")

        cv2.imshow("debug", img_text)
        cv2.waitKey(3)
        

        ###################### FINDING THE POSITION NUMBER ################################
        hsv_forPos = cv2.cvtColor(img_crop, cv2.COLOR_BGR2HSV)
        blurred_forPos = cv2.GaussianBlur(hsv_forPos, (5, 5), 0)
        lower_forPos = np.array([0, 0, 90])
        upper_forPos = np.array([0, 0, 210])
        mask_forPos = cv2.inRange(blurred_forPos, lower_forPos, upper_forPos)

        contours_Pos, _ = cv2.findContours(mask_forPos, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        sortedCont_Pos = sorted(contours_Pos, key=cv2.contourArea, reverse=True)
        cnt_Pos = sortedCont_Pos[0]
        check_areaPos = cv2.contourArea(cnt_Pos)
        if check_areaPos > 7000:
          perimeter_Pos = cv2.arcLength(cnt_Pos, True)
          epsilon_Pos = 0.01*perimeter_Pos
          approx_Pos = cv2.approxPolyDP(cnt_Pos, epsilon_Pos, True)
          # For a portrait image, changes in y are more noticable so add weight to the y value
          # sorts points from descending weighted sum (ie top left, top right, bottom left, bottom right)
          sortedPoints_Pos = sorted(approx_Pos, key=lambda x: x[0, 0]+3*x[0, 1])

          height_forPos, width_forPos = (230, 230)
          # Order: top left, top right, bottom left, bottom right)
          dstPts_Pos = [[0, 0], [width_forPos, 0], [0, height_forPos], [width_forPos, height_forPos]]
          m_Pos = cv2.getPerspectiveTransform(np.float32(sortedPoints_Pos), np.float32(dstPts_Pos))
          # Transform the image
          norm = cv2.warpPerspective(img_crop, m_Pos, (int(width_forPos), int(height_forPos)))
          # cut out the position number
          position = norm[70:170, 120:240]
          ###################### FINDING THE POSITION NUMBER ################################



          height, width = (50,230)
          # referenced https://arccoder.medium.com/straighten-an-image-of-a-page-using-opencv-313182404b06
          # Order: top left, bottom left, top right, bottom right)
          dstPts = [[0, 0], [0, height], [width, 0], [width, height]]
          # Get the transform
          m = cv2.getPerspectiveTransform(np.float32(sortedPoints), np.float32(dstPts))
          # Transform the image
          plate = cv2.warpPerspective(img_crop, m, (int(width), int(height)))

          # # cut out the plate
          # plate_height = 50
          # plate = norm[norm.shape[0]-plate_height:,:]
          

          
          plate_and_pos = np.concatenate( 
            (np.pad(plate,[(0,50),(0,0),(0,0)],mode='constant'),
            position), axis=1)
          # cv2.imshow("image5", norm)
          # cv2.waitKey(3) 
          # cv2.imshow("image6", position)
          # cv2.waitKey(3) 
          cv2.namedWindow("Plate and Position", cv2.WINDOW_NORMAL)
          cv2.imshow("Plate and Position", plate_and_pos)
          cv2.waitKey(3)

















          

          # lower = np.array([98, 110, 2])
          # upper = np.array([120, 255, 200])
          # hsv = cv2.cvtColor(plate, cv2.COLOR_BGR2HSV)
          # plate = cv2.inRange(hsv,lower,upper)
          # # plate = cv2.erode(plate,erode_kernel,iterations=1)
          # # mask = cv2.dilate(mask,dilate_kernel,iterations=1)
          # # plate = cv2.GaussianBlur(plate,(9,9),0)

          # # img = cv2.erode(img,erode_kernel,iterations=2)
          # # cv2.imshow("hsv",hsv)
          # # cv2.waitKey(3)
          # # cv2.imshow("plate",plate)
          # # cv2.waitKey(3)

          # # guess location of characters in order
          # guesses = [plate[:,:60],
          #         plate[:,50:110],
          #         plate[:,130:180],
          #         plate[:,170:]]


          # chars = []
          # # find 4 characters
          # for guess in guesses:
          #   # find contours
          #   contours, _ = cv2.findContours(guess, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            
          #   # first sort the array by area
          #   sortedContours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)

          #   # take the largest contour
          #   cnt = sortedContours[0]

          #   # # Draw contours for verification
          #   # merge = cv2.merge((guess,guess,guess))
          #   # cv2.drawContours(merge,[cnt],0,(0,255,0),3)
          #   # cv2.imshow('cont',merge)
          #   # cv2.waitKey(3)

          #   x,y,w,h = cv2.boundingRect(cnt)
            
          #   ########### HANDLING CASES WITH "merged characters" ################
          #   if x == 0:
          #     x = 10
          #     w = w-x

          #   if w > 36:
          #     w = 36
          #   ########### HANDLING CASES WITH "merged characters" ################

            
          #   # split the images into three parts: ___, character, ____
          #   split = np.hsplit(guess,[x,x+w])
          #   # cv2.imshow('1',split[0])
          #   # cv2.waitKey(3)
          #   # cv2.imshow('2',split[1])
          #   # cv2.waitKey(3)
          #   # cv2.imshow('3',split[2])
          #   # cv2.waitKey(3)
          #   char = split[1]


          #   # zero pad the sides to make it 100px wide
          #   rows, cols = split[1].shape
          #   if cols < 50:
          #     to_add = (50-cols)//2
          #     # deal with the odd case by adding a extra column of zeroes to left/right (random)
          #     if (100-cols)%2 == 1:
          #       rand = randint(0,1)
          #       char = np.pad(split[1], [(0,0),(to_add+rand,to_add+(1-rand))],
          #                     mode='constant',constant_values = 0)
          #     else:
          #       char = np.pad(split[1], [(0,0),(to_add,to_add)],
          #                     mode='constant',constant_values = 0)

          #   if cols > 100:
          #     char = cv2.resize(char,(50,50))


          #   char = cv2.dilate(char,dilate_kernel3,iterations=1)
          #   _,char = cv2.threshold(char,50,255,cv2.THRESH_BINARY)
          #   char = cv2.erode(char,erode_kernel3,iterations=1)
          #   char = cv2.GaussianBlur(char,(5,5),0)
          #   _,char = cv2.threshold(char,70,255,cv2.THRESH_BINARY)



          #   chars.append(char)

          # # read the characters
          # plate_num = ""
          # for c in chars:
          #   img_aug = np.expand_dims(c, axis=0)
          #   predict = self.model.predict(img_aug)
          #   plate_num+=self.allchars[np.argmax(predict)]


          # # process and read the position number
          # _,position = cv2.threshold(position,40,255,cv2.THRESH_BINARY_INV)
          # position = position[:,:,0]
          # position = cv2.dilate(position,dilate_kernel3,iterations = 1)
          # position = cv2.resize(position,(50,50))
          # # position = cv2.GaussianBlur(position,(15,15),0)
          # # _,position = cv2.threshold(position,60,255,cv2.THRESH_BINARY)
          # # read the number
          # img_aug = np.expand_dims(position, axis=0)
          # pos_predict = self.model.predict(img_aug)
          # position_num=self.allchars[np.argmax(pos_predict)]

          # # cv2.imshow("image7", position)
          # # cv2.waitKey(3) 
          
          # output = str('Team5,password,{},{}').format(position_num,plate_num)
          # print(output)






      # cv2.imshow("G0", guesses[0])
      # cv2.waitKey(3)
      # cv2.imshow("G1", guesses[1])
      # cv2.waitKey(3)
      # cv2.imshow("G2", guesses[2])
      # cv2.waitKey(3)
      # cv2.imshow("G3", guesses[3])
      # cv2.waitKey(3)




      # cv2.imshow("L0", chars[0])
      # cv2.waitKey(3)
      # cv2.imshow("L1", chars[1])
      # cv2.waitKey(3)
      # cv2.imshow("L2", chars[2])
      # cv2.waitKey(3)
      # cv2.imshow("L3", chars[3])
      # cv2.waitKey(3)

    



    else:
      print("Can't see plate")



    # cv2.imshow("image4", blurred)
    # cv2.waitKey(3)
    # cv2.imshow("mask", colour_mask)
    # cv2.waitKey(3)
    # cv2.imshow("image2", img_crop)
    # cv2.waitKey(3)

    



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





    return "drive", True


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