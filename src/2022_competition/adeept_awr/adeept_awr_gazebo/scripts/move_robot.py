#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist


# PID control loop to find the linear and rotational movement of the robot based off of this website: http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
def pid(self, kp, ki, kd, target, current, errSum, lastErr, lastTime, saturation, axis):
  # get the time terms
  now = time.time()
  dt = now - lastTime

  # get the error terms
  error = target - current
  errSum += (error * dt)
  dErr = (error - lastErr) / dt

  #compute the output
  output = kp * error + ki * errSum + kd * dErr

  # saturate the output if necessary
  if(output < -saturation):
    output = -saturation
  elif(output > saturation):
    output = saturation

  # remember the necessary terms for the next loop
  if(axis == 'x'):
    self.lastErrX = error
    self.lastTimeX = now
    self.lastOutputX = output
  elif(axis == 'y'):
    self.lastErrY = error
    self.lastTimeY = now
    self.lastOutputY = output

  print("Desired output: " + str(target) + "\tPosition: " + str(current) + "\tOutput: " + str(output))
  return output

class image_converter:

  def __init__(self):
    # Variables for subscribing and publishing
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
    self.drive_pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)

    # Variables for PID control
    self.lastTimeX = time.time()
    self.lastErrX = 0
    self.errSumX = 0
    self.lastOutputX = 0
    self.lastTimeY = time.time()
    self.lastErrY = 0
    self.errSumY = 0
    self.lastOutputY = 0

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Constants
    (rows,cols,channels) = cv_image.shape
    img_height = 200
    move = Twist()
    gaussianKernel = (11, 11)
    threshold = 220
    kpx = 0.02
    kix = 0
    kdx = 0.00001
    saturationx = 1
    targetx = int(cols/2)
    kpy = 0.05
    kiy = 0
    kdy = 0
    saturationy = 1
    targety = rows-150


    # slice bottom portion of image
    img_bot = cv_image[rows - img_height:,:]
    # Convert the frame to a different grayscale
    img_gray = cv2.cvtColor(img_bot, cv2.COLOR_RGB2GRAY)
    # Blur image to reduce noise
    img_blur = cv2.GaussianBlur(img_gray, gaussianKernel, 0)
    # Binarize the image
    _, img_bin = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY)
    cv2.imshow("Binary Image", img_bin)
    # Find the contours of the image
    contours, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # Overlay the contours onto the original image
    # img_cont = cv2.drawContours(cv_image, contours, -1, (0, 0, 0), 1)


    # Check that contours contains at least one contour then on the largest contour, find the centre of its centre
    # Print a circle at the centre of mass on the original image
    if len(contours) > 1:
      areaArray = []
      for _, c in enumerate(contours):
        area = cv2.contourArea(c)
        areaArray.append(area)

      #first sort the array by area
      sortedContours = sorted(zip(areaArray, contours), key=lambda x: x[0], reverse=True)

      #find the 2 largest contours
      maxContour1 = sortedContours[0][1]
      maxContour2 = sortedContours[1][1]
      
      #maxCont = max(contours, key=cv2.contourArea)
      #contMoments = cv2.moments(maxCont)
      contMoments1 = cv2.moments(maxContour1)
      contMoments2 = cv2.moments(maxContour2)
    # Check if the contour contains any pixels
      if contMoments1['m00'] > 0 and contMoments2['m00'] > 0:
        centre1 = (int(contMoments1['m10']/contMoments1['m00']), int(contMoments1['m01']/contMoments1['m00']))
        img_circle = cv2.circle(cv_image, (centre1[0], rows - img_height + centre1[1]), 5, (0, 0, 255), -1)
        # img_line = cv2.line(img_circle1, (0, targety), (800, targety), (0, 0, 255), 1)

        centre2 = (int(contMoments2['m10']/contMoments2['m00']), int(contMoments2['m01']/contMoments2['m00']))
        img_circle = cv2.circle(cv_image, (centre2[0], rows - img_height + centre2[1]), 5, (0, 0, 255), -1)
        
        avg_centre = (int((centre1[0]+centre2[0])/2), int((centre1[1]+centre2[1])/2))
        img_circle = cv2.circle(cv_image, (avg_centre[0], rows - img_height + avg_centre[1]), 5, (0, 0, 255), -1)
        img_line = cv2.line(img_circle, (0, targety), (800, targety), (0, 0, 255), 1)
        
        cv2.imshow("Circle Image",img_line)
        cv2.waitKey(3)


        # Use PID control to determine the rotation and speed of the vehicle
        move.angular.z = pid(self, kpx, kix, kdx, targetx, avg_centre[0], self.errSumX, self.lastErrX, self.lastTimeX, saturationx, 'x')
        move.linear.x = pid(self, kpy, kiy, kdy, targety, avg_centre[1], self.errSumY, self.lastErrY, self.lastTimeY, saturationy, 'y')

        self.timeout = 0



      # elif contMoments1['m00'] == 0:
      #   #TODO: change state to ___

      # elif contMoments2['m00'] == 0:
      #   #TODO: state change 



      else:
        self.timeout += 1

        if self.timeout > 3:
          done = True

    elif len(contours) == 1:
      #TODO: state change (one line)
      _ = 0 #to delete

    # if the line is not detected
    # If the camera loses the line then go slowly with the last rotation determined from PID
    else:
      move.linear.x = 0.75
      move.angular.z = self.lastOutput
      print("Lost line")

      self.timeout += 1
      if self.timeout > 10:
          done = True

    # cv2.imshow("Image window", img_dot)
    # cv2.waitKey(3)

    # Try to publish the movement to the robot, if not display the error
    try:
      self.drive_pub.publish(move)
      print("Published")
 
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