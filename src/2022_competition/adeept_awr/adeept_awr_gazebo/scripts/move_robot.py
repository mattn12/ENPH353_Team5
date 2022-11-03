#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class image_converter:

  def __init__(self):
    self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
      color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

      #converting image frames to binary
      threshold = 90
      _, img_bin = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)

      #draw contours
      contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      contour_color = (0, 255, 0)
      contour_thickness = 5
      img_contours = cv2.drawContours(img_bin, contours, -1, contour_color, contour_thickness)

      #find middle of the line
      M = cv2.moments(img_contours)
      line_x = int(M['m10'] / M['m00'])

      #draw circle
      color = cv2.circle(color, (line_x, 800 - (2*20)), 20, (0,0,255), -1)

    cv2.imshow("Image window", color)
    cv2.waitKey(3)



    try:
      rate = rospy.Rate(2)
      move = Twist()
      move.linear.x = 1
      
      #Controller values
      tol = 40
      P_turn_scale = 0.015
      P_move_scale = 0.0005
      error = (400 - line_x)

      #P controllerer
      if(abs(error) > tol):
        move.linear.x = 0.3 - abs(P_move_scale*error)
        move.angular.z = P_turn_scale * error
      else:
        move.angular.z = 0
      
      #send move instructions to robot
      self.pub.publish(move)
      rate.sleep(10)
    except CvBridgeError as e:
      print(e)

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
