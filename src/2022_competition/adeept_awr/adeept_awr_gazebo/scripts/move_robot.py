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
    self.pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    # slice bottom portion of image
    img_bot = cv_image[rows-100:,:]
    gray = cv2.cvtColor(img_bot, cv2.COLOR_RGB2GRAY)

    #converting image frames to binary
    threshold = 170
    _, img_bin = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)

    #draw contours
    contours, hierarchy = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour_color = (0, 255, 0)
    contour_thickness = 5
    img_contours = cv2.drawContours(img_bin, contours, -1, contour_color, contour_thickness)

    # check if m00 is zero (avoid division by 0)
    # if the line is detected
    if (len(contours)) > 0:
        
        #find middle of the line
        M = cv2.moments(img_contours)

        # check if m00 is zero (avoid division by 0)
        if (M['m00'] > 0):
            self.timeout = 0

            line_x = int(M['m10'] / M['m00'])

            #draw circle
            img_dot = cv2.circle(cv_image, (line_x, rows-100), 5, (0,0,255), -1)

            # #determine which section the dot is in
            # state_index = (int)(10*line_x/shape[1])
            # if state_index >= 10:
            #     state_index = 9
            # state[state_index] = 1

            # img_monitor = cv2.putText(img_dot, str(state),(0,15), cv2.FONT_HERSHEY_COMPLEX,0.4,(255,255,255),1)

        # moment not found
        else:
            self.timeout += 1
            if self.timeout > 10:
                done = True

    # if the line is not detected
    else:
        self.timeout += 1
        if self.timeout > 10:
            done = True

    cv2.imshow("Image window", img_dot)
    cv2.waitKey(3)



    try:
      rate = rospy.Rate(5)
      move = Twist()
      move.linear.x = 0.01
      
      #Controller values
      tol = 40
      P_turn_scale = 0.01
      P_move_scale = 0.002
      error = (cols/2 - line_x)

      #P controllerer
      if(abs(error) > tol):
        move.linear.x = 0.3 - abs(P_move_scale*error)
        move.angular.z = P_turn_scale * error
      else:
        move.angular.z = 0
      
      #send move instructions to robot
      self.pub.publish(move)
      rate.sleep()
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
