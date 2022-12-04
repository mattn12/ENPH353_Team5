#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import time
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# from geometry_msgs.msg import Twist

# Pants (H,S,V):
# (204, 27, 61) - (211, 61, 15)



class avoid_ped:

    def __init__(self):
        self.move = (0.0, 0.0)
    
    def run_cross_walk(self, cv_image):
        # Constants
        state = "cross_walk"
        colourMin = (100, 30, 0)
        colourMax = (110, 200, 255)
        (rows,cols,channels) = cv_image.shape
        # Speed(forward/back, angular)
        self.move = (0.0, 0.0)

        ## Check the state
        # state = self.state_change(cv_image)
        # if state != "cross_walk":
        #     return state, (0.0,0.0)

        img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, colourMin, colourMax)
        # Do more image processing to remove noise
        img_ero = cv2.erode(img_mask, None, iterations = 2)
        img_dil = cv2.dilate(img_ero, None, iterations = 2)
        cv2.imshow("Dil", img_dil)
        cv2.waitKey(3)

        return self.move





#   def run_drive(self, cv_image):
    



def main(args):
    rospy.init_node('robot_driver', anonymous=True)
    ap = avoid_ped()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Keyboard interrupt")

if __name__ == '__main__':
    main(sys.argv)