#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import time
from numpy import copy
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# from geometry_msgs.msg import Twist



class avoid_ped:

    def __init__(self):
        self.has_gone = False
        self.start_time = 0
    
    def run_cross_walk(self, cv_image):
        move = (0.0, 0.0)
        state = "cross_walk"
        go_timeout = 1.1
        (rows,cols,channels) = cv_image.shape

        move_time = time.time() - self.start_time
        if self.has_gone == True and move_time < go_timeout:
            print("\n\nMoving!!\n\n")
            strtime = str(move_time)
            img_text = cv2.putText(cv_image, strtime, (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            move = (0.5, 0.0)

        elif self.has_gone == True and move_time >= go_timeout:
            print("\n\nDone Moving!!\n\n")
            # RESET has_gone (VERY IMPORTANT FOR THE NEXT XWALK)
            self.has_gone = False
            img_text = cv2.putText(cv_image, "Done Cross", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            move = (0.5, 0.0)
            state = "drive"

        else:
            # Constants"
            colourMin = (100, 30, 0)
            colourMax = (110, 200, 255)
            bound1 = int((2 / 5) * cols)
            bound2 = int((3 / 5) * cols)

            # Speed(forward/back, angular)

            ## Check the state
            # state = self.state_change(cv_image)
            # if state != "cross_walk":
            #     return state, (0.0,0.0)

            img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            img_mask = cv2.inRange(img_hsv, colourMin, colourMax)
            # Do more image processing to remove noise
            img_ero = cv2.erode(img_mask, None, iterations = 2)
            img_dil = cv2.dilate(img_ero, None, iterations = 2)
            # cv2.imshow("Dil", img_dil)
            # cv2.waitKey(3)

            contours, _ = cv2.findContours(img_dil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            # Check if there are any contours
            if len(contours) > 0:
                # Sort Contours
                sortedContours = sorted(contours, key=cv2.contourArea, reverse=True)

                # Get largest contour 
                maxContMom = cv2.moments(sortedContours[0])
                # Draw the contour on the original image
                img_cont = cv2.drawContours(cv_image, sortedContours, 0, (255, 0, 0), 2)
                img_text = cv2.putText(img_cont, "Cross Walk", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

                # Make sure the contour is not zero volume
                if maxContMom['m00'] > 0:
                    # Get centroid of contour
                    centre = (int(maxContMom['m10']/maxContMom['m00']), int(maxContMom['m01']/maxContMom['m00']))

                    img_line = cv2.line(img_cont, (bound1, 0), (bound1, rows), (255, 255, 255), 1)
                    img_line = cv2.line(img_line, (bound2, 0), (bound2, rows), (255, 255, 255), 1)

                    # If the centre of the contour is in the centre third of the screen, go forward
                    # Pedestrian is in the centre, move forward slow enough that they will cross and then
                    #   you will pass the sidewalf before they cross again
                    if centre[0] > bound1 and centre[0] < bound2:
                        img_text = cv2.putText(img_cont, "GO!!!", (int(cols / 2), 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                        self.has_gone = True
                        time.sleep(0.5)
                        move = (0.45, 0)
                        self.start_time = time.time()

        # img_text = copy(cv_image)
        # cv2.imshow("Debug Image", img_text)
        # cv2.waitKey(3)
        return state, move
 



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