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
  if(axis == 'NormAng'):
    self.lastErrNormAng = error
    self.lastTimeNormAng = now
    self.LastOutputNormAng = output
  elif(axis == 'NormLine'):
    self.LastErrNormLine = error
    self.LastTimeNormLine = now
    self.LastOutputNormLine = output
  elif(axis == 'OneAng'):
    self.LastErrOneAng = error
    self.LastTimeOneAng = now
    self.LastOutputOneAng = output

  print("Desired output: " + str(target) + "\tPosition: " + str(current) + "\tOutput: " + str(output))
  return output

class image_converter:

  def __init__(self):
    # Variables for subscribing and publishing
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.callback)
    self.drive_pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)
    self.license_pub = rospy.Publisher("/license_plate", String, queue_size=1)

    # Variables for PID control
    self.lastTimeNormAng = time.time()
    self.lastErrNormAng = 0
    self.errSumNormAng = 0
    self.LastOutputNormAng = 0
    self.LastTimeNormLine = time.time()
    self.LastErrNormLine = 0
    self.errSumNormLine = 0
    self.LastOutputNormLine = 0
    self.LastTimeOneAng = time.time()
    self.LastErrOneAng = 0
    self.errSumOneAng = 0
    self.LastOutputOneAng = 0

    self.timeout = 0

    self.startRun = True
    self.endRun = False

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Constants
    (rows,cols,channels) = cv_image.shape
    crop_height = 200
    move = Twist()
    gaussianKernel = (11, 11)
    threshold = 220
    kpAng = 0.02
    kiAng = 0
    kdAng = 0.00001
    saturationAng = 2
    targetAng = int(cols/2)
    kpLine = 0.0005
    kiLine = 0
    kdLine = 0
    saturationLine = 1
    targetLine = rows-100
    kpAngOne =0.06
    kiAngOne = 0
    kdAngOne = 0.00001
    saturationAngOne = 2


    # Code for starting the timer/sending plates

    if self.startRun:
        output = str('Team5,password,0,ABCD')
        self.startRun = False

        try:
          self.license_pub.publish(output)
        except CvBridgeError as e:
          print(e)

    # Test code
    # elif self.testPlate:
    #     output = str('Team5,password,{},{}').format(plate_num,plate)
    #     self.testPlate = False

    # else:
    #     output = ''
    
    



    # slice bottom portion of image
    img_bot = cv_image[rows - crop_height:,:]
    # Convert the frame to a different grayscale
    img_gray = cv2.cvtColor(img_bot, cv2.COLOR_RGB2GRAY)
    # Blur image to reduce noise
    img_blur = cv2.GaussianBlur(img_gray, gaussianKernel, 0)
    # Binarize the image
    _, img_bin = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY)
    cv2.imshow("Binary Image", img_bin)
    # Do more image processing to remove noise
    img_ero = cv2.erode(img_bin, None, iterations = 2)
    img_dil = cv2.dilate(img_ero, None, iterations = 2)
    cv2.imshow("Noise Suppressed", img_dil)
    # Find the contours of the image
    contours, _ = cv2.findContours(img_dil, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
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
        img_circle = cv2.circle(cv_image, (centre1[0], rows - crop_height + centre1[1]), 5, (0, 0, 255), -1)
        # img_line = cv2.line(img_circle1, (0, targetLine), (800, targetLine), (0, 0, 255), 1)

        centre2 = (int(contMoments2['m10']/contMoments2['m00']), int(contMoments2['m01']/contMoments2['m00']))
        img_circle = cv2.circle(cv_image, (centre2[0], rows - crop_height + centre2[1]), 5, (0, 0, 255), -1)
        
        avg_centre = (int((centre1[0]+centre2[0])/2), int((centre1[1]+centre2[1])/2))
        img_circle = cv2.circle(cv_image, (avg_centre[0], rows - crop_height + avg_centre[1]), 5, (0, 0, 255), -1)
        img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), (0, 0, 255), 1)
        img_line = cv2.line(img_line, (targetAng, 0), (targetAng, rows), (0, 0, 255), 1)
        
        cv2.imshow("Circle Image",img_line)
        cv2.waitKey(3)


        # Use PID control to determine the rotation and speed of the vehicle
        move.angular.z = pid(self, kpAng, kiAng, kdAng, targetAng, avg_centre[0], self.errSumNormAng, self.lastErrNormAng, self.lastTimeNormAng, saturationAng, 'NormAng')
        move.linear.x = pid(self, kpLine, kiLine, kdLine, targetLine, avg_centre[1], self.errSumNormLine, self.LastErrNormLine, self.LastTimeNormLine, saturationLine, 'NormLine')

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
      # _ = 0 #to delete
      side_Offset = 400

      maxCont = max(contours, key=cv2.contourArea)
      contMoments = cv2.moments(maxCont)
      contCentre = (int(contMoments['m10']/contMoments['m00']), int(contMoments['m01']/contMoments['m00']))

      if contCentre[0] < int(cols/2):
        centre = (contCentre[0] + side_Offset, contCentre[1])
      elif contCentre[0] > int(cols/2):
        centre = (contCentre[0] - side_Offset, contCentre[1])
      else:
        centre = contCentre

      img_circle = cv2.circle(cv_image, (centre[0], rows - crop_height + centre[1]), 5, (0, 0, 255), -1)
      img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), (0, 0, 255), 1)
      img_line = cv2.line(img_line, (targetAng, 0), (targetAng, rows), (0, 0, 255), 1)
      cv2.imshow("Circle Image",img_line)
      cv2.waitKey(3)

      # Use PID control to determine the rotation and speed of the vehicle
      move.angular.z = pid(self, kpAngOne, kiAngOne, kdAngOne, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturationAngOne, 'OneAng')
      # move.linear.x = pid(self, kpLine, kiLine, kdLine, targetLine, centre[1], self.errSumNormLine, self.LastErrNormLine, self.LastTimeNormLine, saturationLine, 'y')
      move.linear.x = 0.5
      print("Lost line")

    # if the line is not detected
    # If the camera loses the line then go slowly with the last rotation determined from PID
    else:
      move.linear.x = 0.75
      move.angular.z = self.LastOutputNormLine
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