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
    self.alpha_model = models.load_model('src/enph353_comp_controller/src/plate_models/alpha_model.h5')
    self.num_model = models.load_model('src/enph353_comp_controller/src/plate_models/num_model.h5')
    self.allalpha = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    self.allnum = "0123456789"

  def read_plate(self,plate,position):
    orig = np.copy(plate)
    erode_kernel3 = np.ones((3,3))
    dilate_kernel3 = np.ones((3, 3))

    lower = np.array([98, 110, 2])
    upper = np.array([120, 255, 200])
    hsv = cv2.cvtColor(plate, cv2.COLOR_BGR2HSV)
    plate = cv2.inRange(hsv, lower, upper)

    # guess location of characters in order
    guesses = [plate[:,:60],
            plate[:,50:110],
            plate[:,130:180],
            plate[:,170:]]

    chars = []
    # find 4 characters
    for guess in guesses:
      # find contours
      contours, _ = cv2.findContours(guess, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
      
      # first sort the array by area
      sortedContours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)

      # take the largest contour
      cnt = sortedContours[0]

      x,y,w,h = cv2.boundingRect(cnt)
      
      ########### HANDLING CASES WITH "merged characters" ################
      if x == 0:
        x = 10
        w = w-x

      if w > 36:
        w = 36
      ########### HANDLING CASES WITH "merged characters" ################

      # split the images into three parts: ___, character, ____
      split = np.hsplit(guess,[x,x+w])
      char = split[1]

      # zero pad the sides to make it 100px wide
      rows, cols = split[1].shape
      if cols < 50:
        to_add = (50-cols)//2
        # deal with the odd case by adding a extra column of zeroes to left/right (random)
        if (100-cols)%2 == 1:
          rand = randint(0,1)
          char = np.pad(split[1], [(0,0),(to_add+rand,to_add+(1-rand))],
                        mode='constant',constant_values = 0)
        else:
          char = np.pad(split[1], [(0,0),(to_add,to_add)],
                        mode='constant',constant_values = 0)
      if cols > 100:
        char = cv2.resize(char,(50,50))

      char = cv2.dilate(char,dilate_kernel3,iterations=1)
      _,char = cv2.threshold(char,50,255,cv2.THRESH_BINARY)
      char = cv2.erode(char,erode_kernel3,iterations=1)
      char = cv2.GaussianBlur(char,(5,5),0)
      _,char = cv2.threshold(char,70,255,cv2.THRESH_BINARY)
      chars.append(char)

    
    # read the characters
    plate_num = ""
    for i in range(4):
      img_aug = np.expand_dims(chars[i], axis=0)
      if i < 2:
        predict = self.alpha_model.predict(img_aug)
        plate_num+=self.allalpha[np.argmax(predict)]
      else:
        predict = self.num_model.predict(img_aug)
        plate_num += self.allnum[np.argmax(predict)]


    # process and read the position number
    _,position = cv2.threshold(position,40,255,cv2.THRESH_BINARY_INV)
    position = position[:,:,0]
    position = cv2.dilate(position,dilate_kernel3,iterations = 1)
    position = cv2.resize(position,(50,50))
    
    
    # read the number
    img_aug = np.expand_dims(position, axis=0)
    pos_predict = self.num_model.predict(img_aug)
    position_num=self.allnum[np.argmax(pos_predict)]

    
    output = str('Team5,password,{},{}').format(position_num,plate_num)
    cv2.imwrite(os.path.join("/home/matthew/Plate Images/"+plate_num+".png"), orig)

    
    return "drive", True, output    


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
