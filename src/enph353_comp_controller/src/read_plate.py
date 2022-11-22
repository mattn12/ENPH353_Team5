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

class image_converter:

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
    matching_tol = 1
    img_height = 400
    threshold = 90
    sharpen_kernel = np.array([[0,-1,0],[-1,5,-1],[0,-1,0]])
    gaussianKernel = (5, 5)
    
    (rows,cols,channels) = cv_image.shape

    img_crop = cv_image[rows - img_height:,:]
    gray = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, gaussianKernel, 0)
    # gray = cv2.filter2D(src=img_crop,ddepth=-1,kernel=sharpen_kernel)


    #figure out wtf this does
    index_params = dict(algorithm=0, trees=5) 
    search_params = dict()
    flann = cv2.FlannBasedMatcher(index_params,search_params)


    # read template image
    path = os.path.join(os.path.dirname(__file__),'P_template (1).png')
    img = cv2.imread(path,0)

    # print(os.path.exists(path))
    
    # Find features of template image
    sift = cv2.xfeatures2d.SIFT_create()
    kp_img, desc_img = sift.detectAndCompute(img, None)
    img = cv2.drawKeypoints(img, kp_img, img)


    # Find features of frame image
    sift = cv2.xfeatures2d.SIFT_create()
    kp_gray, desc_gray = sift.detectAndCompute(gray, None)
    gray = cv2.drawKeypoints(gray, kp_gray, gray)


    # Find matches between template and frame
    matches = flann.knnMatch(desc_img, desc_gray, k=2)	
    good_points = []
    for m, n in matches:
        if m.distance < matching_tol * n.distance:
            good_points.append(m)


    # draw lines for matching keypoint
    matching = cv2.drawMatches(img,kp_img,gray,kp_gray,good_points,gray)

    # Homography
    query_pts = np.float32([kp_img[m.queryIdx].pt for m in good_points]).reshape(-1, 1, 2)
    train_pts = np.float32([kp_gray[m.trainIdx].pt for m in good_points]).reshape(-1, 1, 2)
    matrix, mask = cv2.findHomography(query_pts, train_pts, cv2.RANSAC, 5.0)
    matches_mask = mask.ravel().tolist()

    # Perspective transform
    h, w = img.shape[0:2]
    pts = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)
    dst = cv2.perspectiveTransform(pts, matrix)

    #draw lines around object
    homography = cv2.polylines(img_crop, [np.int32(dst)], True, (0, 255, 0), 3)

    cv2.imshow("image1", homography)
    cv2.waitKey(3)

    cv2.imshow("image2", matching)
    cv2.waitKey(3)

    plate_num = 1
    plate = 'WXYZ'




    # # Code for starting the timer/sending plates

    # if self.startRun:
    #     output = str('Team5,password,0,ABCD')
    #     self.startRun = False

    # # Test code
    # elif self.testPlate:
    #     output = str('Team5,password,{},{}').format(plate_num,plate)
    #     self.testPlate = False

    # else:
    #     output = ''
    

    # try:
    #   self.license_pub.publish(output)
    # except CvBridgeError as e:
    #   print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)