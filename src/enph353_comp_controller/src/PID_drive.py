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

from random import randint

from geometry_msgs.msg import Twist


class PID_driver:

  def __init__(self):
    
    ##################### UNCOMMENT FOR INDIV. TESTING #####################
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.drive)
    self.license_pub = rospy.Publisher("/license_plate", String, queue_size=1)
    self.drive_pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)
    self.move = Twist()
    ##################### UNCOMMENT FOR INDIV. TESTING #####################
    
    self.targetY = 100
    self.oneOffset = 550
    self.lastmove = 0
    self.oneErrorScale = 0.16

  def drive(self,cv_image):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(cv_image,"bgr8")
    except CvBridgeError as e:
        print(e)
    
    def process_img(cv_image):
      # Variables
        gaussianKernel = (5, 5)
        threshold = 220
        colourMin = (120, 125, 125)
        colourMax = (255, 135, 135)

        # Get LAB
        img_lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)

        # Get range of colours
        img_mask = cv2.inRange(img_lab, colourMin, colourMax)
        # cv2.imshow("LAB Mask", img_mask)
        # cv2.waitKey(3)

        # lab_filter = cv2.inRange(cv2.cvtColor(cv_image,cv2.COLOR_BGR2LAB),(120,125,125),(255,135,135))
        # cv2.imshow("LAB", img_lab)
        # cv2.waitKey(3)
        # cv2.imshow("LAB_filter", lab_filter)
        # cv2.waitKey(3)

        # Blur image
        img_blur = cv2.GaussianBlur(img_mask, gaussianKernel, 0)
        # Binarize the image
        # _, img_bin = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY)
        # Do more image processing to remove noise
        img_ero = cv2.erode(img_blur, np.ones((5,5)), iterations=2)
        img_dil = cv2.dilate(img_ero, np.ones((5,5)), iterations=3)
        # cv2.imshow("Noise Suppressed", img_dil)
        cv2.waitKey(3)

        return img_dil

    # get image from robot camera
    (rows, cols, channels) = cv_image.shape
    img_height = 200
    img_crop = cv_image[rows - img_height:,:]

    processed = process_img(img_crop)

    img_draw = np.copy(img_crop)
    contours, _ = cv2.findContours(processed, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
    cv2.drawContours(img_draw, contours, -1, (255, 0, 0), 3)
    cv2.line(img_draw, (cols//2, 0), (cols//2, img_height), (0,0,255), 1)

    print(self.move)

    if len(contours) > 0:
        sortedCont = sorted(contours,key=cv2.contourArea,reverse=True)

        if len(contours) == 1:
            M1 = cv2.moments(sortedCont[0])
            if M1['m00'] > 0:
                center1 = (int(M1['m10'] / M1['m00']), int(M1['m01'] / M1['m00']))

                cv2.circle(img_draw, (center1[0], center1[1]), 5, (0, 0, 255), -1)
                
                abs_errorX = self.oneErrorScale*np.abs((cols//2)-center1[0])

                # Line on left
                if center1[0] < (cols//2):
                    targetX = (cols//2)+abs_errorX
                # Line on right
                else:
                    targetX = (cols//2)-abs_errorX

                errorX = (targetX-cols//2)
                errorY = center1[1]

                cv2.circle(img_draw, (center1[0], center1[1]), 5, (0, 0, 255), -1)


                self.move.linear.x, self.move.angular.z = self.onePID(errorX, errorY)


        elif len(contours) == 2:
            M1 = cv2.moments(sortedCont[0])
            M2 = cv2.moments(sortedCont[1])

            if M1['m00'] > 0 and M2['m00'] > 0:
                center1 = (int(M1['m10'] / M1['m00']), int(M1['m01'] / M1['m00']))
                center2 = (int(M2['m10'] / M2['m00']), int(M2['m01'] / M2['m00']))

                cv2.circle(img_draw, (center1[0], center1[1]), 5, (0, 0, 255), -1)
                cv2.circle(img_draw, (center2[0], center2[1]), 5, (0, 0, 255), -1)

                avgX = (center1[0]+center2[0])//2
                avgY = (center1[1]+center2[1])//2

                errorX = (cols//2)-avgX
                errorY = self.targetY-avgY

                cv2.circle(img_draw, (avgX, 70), 5, (0, 0, 255), -1)


                self.move.linear.x, self.move.angular.z = self.twoPID(errorX,errorY)

    else:
        self.move.linear.x = -0.2
        self.move.angular.z = 3*self.lastmove[1]   




    cv2.imshow("aaaa",img_draw)
    cv2.waitKey(3)

    try:
      self.drive_pub.publish(self.move)
      print(str('Published: LinX = {}, AngZ = {}').format(self.move.linear.x,self.move.angular.z))

    except CvBridgeError as e:
      print(e)

  def onePID(self, errorX, errorY):
    p = 0.035

    self.move.linear.x = 0.16
    self.move.angular.z = p*errorX

    output = self.move.linear.x, self.move.angular.z
    self.lastmove = output

    return output

  def twoPID(self,errorX,errorY):
    p = 0.02
    self.move.angular.z = p*errorX

    sat = 1.5

    if self.move.angular.z > sat:
        self.move.angular.z = sat
    if self.move.angular.z < -sat:
        self.move.angular.z = -sat

    self.move.linear.x = 0.34
    if np.abs(errorY) > 10 and np.abs(errorY) < 20:
        self.move.linear.x = 0.39
        self.move.angular.z = 2/3*self.move.angular.z
        print("\n\n\n\n\nTwo errorY: "+str(errorY))

    
    
   
    
    output = self.move.linear.x, self.move.angular.z
    self.lastmove = output

    return output

    


def main(args):
  
  rospy.init_node('plate_reader', anonymous=True)
  PID = PID_driver()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)