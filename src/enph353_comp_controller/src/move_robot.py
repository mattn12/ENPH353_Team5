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

  print("Type: " + axis + "\tDesired output: " + str(target) + "\tPosition: " + str(current) + "\tOutput: " + str(output))
  return output

def reset_variables(self):
  
  self.lastErrNormAng = 0
  self.errSumNormAng = 0
  self.LastOutputNormAng = 0

  self.LastErrNormLine = 0
  self.errSumNormLine = 0
  self.LastOutputNormLine = 0
  
  self.LastErrOneAng = 0
  self.errSumOneAng = 0
  self.LastOutputOneAng = 0

class robot_driver:

  def __init__(self):

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

    # Timing variables
    # self.startRun = True
    # self.endRun = False

    # Other Variables
    self.move = (0.0, 0.0)
    # self.lastmove = (0.0, 0.0)
    self.dotColour = (150, 150, 150)



  def run_drive(self, cv_image):
    # Constants
    state = "drive"
    (rows,cols,channels) = cv_image.shape
    topcrop_height = 200
    midcrop_height = 100
    side_Offset = 400
    speed = 0.25
    # Speed(forward/back, angular)
    self.move = (0.0, 0.0)
    gaussianKernel = (11, 11)
    threshold = 220
    kpAng = 0.015
    kiAng = 0
    kdAng = 0.00001
    saturationAng = 2
    targetAng = int(cols/2)
    kpLine = 0.0004
    kiLine = 0
    kdLine = 0
    saturationLine = 1
    targetLine = rows-100
    kpAngOne = 0.05
    kiAngOne = 0
    kdAngOne = 0.00001
    saturationAngOne = 2

    ## Check the state
    state = self.state_change(cv_image)
    if state != "drive":
      return state, (0.0,0.0)

    ## Image Processing

    processed_img = self.process_img(cv_image)
    #todo Determine if we need to change state


    # slice bottom portion of image
    img_top = processed_img[rows - topcrop_height:rows - midcrop_height,:]
    # cv2.imshow("TopCropped", img_top)
    # cv2.waitKey(3)
    img_bot = processed_img[rows - midcrop_height:,:]
    # cv2.imshow("BottomCropped", img_bot)
    # cv2.waitKey(3)
    # # Convert the frame to a different grayscale
    # img_gray = cv2.cvtColor(img_bot, cv2.COLOR_RGB2GRAY)
    # # Blur image to reduce noise
    # img_blur = cv2.GaussianBlur(img_gray, gaussianKernel, 0)
    # # Binarize the image
    # _, img_bin = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY)
    # cv2.imshow("Binary Image", img_bin)
    # cv2.waitKey(3)

    # # Do more image processing to remove noise
    # img_ero = cv2.erode(img_bin, None, iterations = 2)
    # img_dil = cv2.dilate(img_ero, None, iterations = 2)
    # # cv2.imshow("Noise Suppressed", img_dil)

    # Find the contours of the image
    topcontours, _ = cv2.findContours(img_top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    botcontours, _ = cv2.findContours(img_bot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    # Overlay the contours onto the original image
    # img_cont = cv2.drawContours(cv_image, contours, -1, (0, 0, 0), 1)


    # If there are at least 2 contours in top and bottom, line follow them
    if len(topcontours) > 1 and len(botcontours) > 1:
      # areaArray = []
      # for _, c in enumerate(topcontours):
      #   area = cv2.contourArea(c)
      #   areaArray.append(area)

      # #first sort the array by area
      # sortedContours = sorted(zip(areaArray, topcontours), key=lambda x: x[0], reverse=True)
      topSortedCont = self.sortContours(topcontours)
      botSortedCont = self.sortContours(botcontours)

      #find the 2 largest contours
      # topMaxCont1 = topSortedCont[0][1]
      # topMaxCont2 = topSortedCont[1][1]
      # botMaxCont1 = botSortedCont[0][1]
      # botMaxCont2 = botSortedCont[1][1]
      
      #maxCont = max(contours, key=cv2.contourArea)
      #contMoments = cv2.moments(maxCont)
      topContMom1 = cv2.moments(topSortedCont[0][1])
      topContMom2 = cv2.moments(topSortedCont[1][1])
      botContMom1 = cv2.moments(botSortedCont[0][1])
      botContMom2 = cv2.moments(botSortedCont[1][1])

    # Check if the contour contains any pixels
      if topContMom1['m00'] > 0 and topContMom2['m00'] > 0 and botContMom1['m00'] > 0 and botContMom2['m00'] > 0:
        # Find and draw centre of biggest top contour
        topcentre1 = (int(topContMom1['m10']/topContMom1['m00']), int(topContMom1['m01']/topContMom1['m00']))
        img_circle = cv2.circle(processed_img, (topcentre1[0], rows - topcrop_height + topcentre1[1]), 5, self.dotColour, -1)

        # Find and draw centre of second biggest top contour
        topcentre2 = (int(topContMom2['m10']/topContMom2['m00']), int(topContMom2['m01']/topContMom2['m00']))
        img_circle = cv2.circle(img_circle, (topcentre2[0], rows - topcrop_height + topcentre2[1]), 5, self.dotColour, -1)
        
        # Find and draw centre of biggest bottom contour
        botcentre1 = (int(botContMom1['m10']/botContMom1['m00']), int(botContMom1['m01']/botContMom1['m00']))
        img_circle = cv2.circle(img_circle, (botcentre1[0], rows - midcrop_height + botcentre1[1]), 5, self.dotColour, -1)

        # Find and draw centre of biggest bottom contour
        botcentre2 = (int(botContMom2['m10']/botContMom2['m00']), int(botContMom2['m01']/botContMom2['m00']))
        img_circle = cv2.circle(img_circle, (botcentre2[0], rows - midcrop_height + botcentre2[1]), 5, self.dotColour, -1)

        if topcentre1[0] < topcentre2[0]:
          topleft, topright = topcentre1, topcentre2
        else:
          topleft, topright = topcentre2, topcentre1

        if botcentre1[0] < botcentre2[0]:
          botleft, botright = botcentre1, botcentre2
        else:
          botleft, botright = botcentre2, botcentre1

        # topleft, topright = topcentre1, topcentre2 if topcentre1[0] < topcentre2[0] else topcentre2, topcentre1
        # botleft, botright = botcentre1, botcentre2 if botcentre1[0] < botcentre2[0] else botcentre2, botcentre1

        # Follow Centre of two lines
        if topleft[0] > botleft[0] and topright[0] < botright[0]:
          ave_centre = (int((topleft[0] + botleft[0] + topright[0] + botright[0]) / 4), int((topleft[1] + botleft[1] + topright[1] + botright[1]) / 4))
          img_circle = cv2.circle(img_circle, (ave_centre[0], 200), 5, self.dotColour, -1)
          img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
          # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
          cv2.imshow("Debug Image", img_line)
          cv2.waitKey(3)

          print("Reguler 2 lines")
          angular = pid(self, kpAng, kiAng, kdAng, targetAng, ave_centre[0], self.errSumNormAng, self.lastErrNormAng, self.lastTimeNormAng, saturationAng, 'NormAng')
          self.move = (speed, angular)

        # Follow right line
        elif topleft[0] < botleft[0] and topright[0] < botright[0]:
          
          rightCentre = (int((topright[0] + botright[0]) / 2), int((topright[1] + botright[1]) / 2))

          centre = (rightCentre[0] - side_Offset, rightCentre[1])

          img_circle = cv2.circle(img_circle, (centre[0], 200), 5, self.dotColour, -1)
          # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
          img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
          cv2.imshow("Debug Image",img_line)
          cv2.waitKey(3)

          # Use PID control to determine the rotation and speed of the vehicle
          angular = pid(self, kpAngOne, kiAngOne, kdAngOne, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturationAngOne, 'OneAng')
          self.move = (speed, angular)
          print("Following Right line")

        # Follow left line
        elif topleft[0] > botleft[0] and topright[0] > botright[0]:
          
          leftCentre = (int((topleft[0] + botleft[0]) / 2), int((topleft[1] + botleft[1]) / 2))

          centre = (leftCentre[0] + side_Offset, leftCentre[1])

          img_circle = cv2.circle(img_circle, (centre[0], 200), 5, self.dotColour, -1)
          # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
          img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
          cv2.imshow("Debug Image",img_line)
          cv2.waitKey(3)

          # Use PID control to determine the rotation and speed of the vehicle
          angular = pid(self, kpAngOne, kiAngOne, kdAngOne, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturationAngOne, 'OneAng')
          self.move = (speed, angular)
          print("Following Left line")

        # avg_centre = (int((centre1[0]+centre2[0])/2), int((centre1[1]+centre2[1])/2))
        # img_circle = cv2.circle(cv_image, (avg_centre[0], rows - crop_height + avg_centre[1]), 5, (0, 0, 255), -1)
        # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), (0, 0, 255), 1)
        # img_line = cv2.line(img_line, (targetAng, 0), (targetAng, rows), (0, 0, 255), 1)
        
        # cv2.imshow("Circle Image",img_line)
        # cv2.waitKey(3)


        # Use PID control to determine the rotation and speed of the vehicle
        # angular = pid(self, kpAng, kiAng, kdAng, targetAng, avg_centre[0], self.errSumNormAng, self.lastErrNormAng, self.lastTimeNormAng, saturationAng, 'NormAng')
        # linear = pid(self, kpLine, kiLine, kdLine, targetLine, avg_centre[1], self.errSumNormLine, self.LastErrNormLine, self.LastTimeNormLine, saturationLine, 'NormLine')
        # self.move = (linear, angular)

        



      # elif contMoments1['m00'] == 0:
      #   #TODO: change state to ___

      # elif contMoments2['m00'] == 0:
      #   #TODO: state change 


      # The area of a contour is zero
      else:
        print("Line's Area is zero")

    # There is one contour in the top, follow the side with the top contour
    elif len(topcontours) == 1 and len(botcontours) > 1:
      #TODO: state change (one line)
      # side_Offset = 400

      botSortedCont = self.sortContours(botcontours)

      topContMom = cv2.moments(topcontours[0][1])
      botContMom1 = cv2.moments(botSortedCont[0][1])
      botContMom2 = cv2.moments(botSortedCont[1][1])

      # Find and draw centre of top contour
      topcentre = (int(topContMom['m10']/topContMom['m00']), int(topContMom['m01']/topContMom['m00']))
      img_circle = cv2.circle(processed_img, (topcentre[0], rows - midcrop_height + topcentre[1]), 5, self.dotColour, -1)

      # Find and draw centre of biggest bottom contour
      botcentre1 = (int(botContMom1['m10']/botContMom1['m00']), int(botContMom1['m01']/botContMom1['m00']))
      img_circle = cv2.circle(img_circle, (botcentre1[0], rows - midcrop_height + botcentre1[1]), 5, self.dotColour, -1)

      # Find and draw centre of biggest bottom contour
      botcentre2 = (int(botContMom2['m10']/botContMom2['m00']), int(botContMom2['m01']/botContMom2['m00']))
      img_circle = cv2.circle(img_circle, (botcentre2[0], rows - midcrop_height + botcentre2[1]), 5, self.dotColour, -1)

      if botcentre1[0] < botcentre2[0]:
        botleft, botright = botcentre1, botcentre2
      else:
        botleft, botright = botcentre2, botcentre1

      # botleft, botright = botcentre1, botcentre2 if botcentre1[0] < botcentre2[0] else botcentre2, botcentre1

      # Follow Left Side
      if topcentre[0] <= int(cols / 2):
        leftCentre = (int((topcentre[0] + botleft[0]) / 2), int((topcentre[1] + botleft[1]) / 2))

        centre = (leftCentre[0] + side_Offset, leftCentre[1])

        img_circle = cv2.circle(img_circle, (centre[0], 200), 5, self.dotColour, -1)
        # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
        img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
        cv2.imshow("Debug Image",img_line)
        cv2.waitKey(3)

        # Use PID control to determine the rotation and speed of the vehicle
        angular = pid(self, kpAngOne, kiAngOne, kdAngOne, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturationAngOne, 'OneAng')
        self.move = (speed, angular)
        print("Following Left line")

      # Follow Right Side
      elif topcentre[0] > int(cols / 2):
        rightCentre = (int((topcentre[0] + botright[0]) / 2), int((topcentre[1] + botright[1]) / 2))

        centre = (rightCentre[0] - side_Offset, rightCentre[1])

        img_circle = cv2.circle(img_circle, (centre[0], 200), 5, self.dotColour, -1)
        # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
        img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
        cv2.imshow("Debug Image",img_line)
        cv2.waitKey(3)

        # Use PID control to determine the rotation and speed of the vehicle
        angular = pid(self, kpAngOne, kiAngOne, kdAngOne, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturationAngOne, 'OneAng')
        self.move = (speed, angular)
        print("Following Right line")

    # There is one contour in the bottom, follow the side with the bottom contour
    elif len(topcontours) > 1 and len(botcontours) == 1:
      #TODO: state change (one line)
      # side_Offset = 400

      topSortedCont = self.sortContours(topcontours)

      topContMom1 = cv2.moments(topSortedCont[0][1])
      topContMom2 = cv2.moments(topSortedCont[1][1])
      botContMom = cv2.moments(botcontours[0][1])

      # Find and draw centre of biggest top contour
      topcentre1 = (int(topContMom1['m10']/topContMom1['m00']), int(topContMom1['m01']/topContMom1['m00']))
      img_circle = cv2.circle(processed_img, (topcentre1[0], rows - midcrop_height + topcentre1[1]), 5, self.dotColour, -1)

      # Find and draw centre of second top contour
      topcentre2 = (int(topContMom2['m10']/topContMom2['m00']), int(topContMom2['m01']/topContMom2['m00']))
      img_circle = cv2.circle(img_circle, (topcentre2[0], rows - midcrop_height + topcentre2[1]), 5, self.dotColour, -1)

      # Find and draw centre of biggest bottom contour
      botcentre = (int(botContMom['m10']/botContMom['m00']), int(botContMom['m01']/botContMom['m00']))
      img_circle = cv2.circle(img_circle, (botcentre[0], rows - midcrop_height + botcentre[1]), 5, self.dotColour, -1)

      if topcentre1[0] < topcentre2[0]:
        topleft, topright = topcentre1, topcentre2
      else:
        topleft, topright = topcentre2, topcentre1

      # botleft, botright = botcentre1, botcentre2 if botcentre1[0] < botcentre2[0] else botcentre2, botcentre1

      # Follow Left Side
      if botcentre[0] <= int(cols / 2):
        leftCentre = (int((topleft[0] + botcentre[0]) / 2), int((topleft[1] + botcentre[1]) / 2))

        centre = (leftCentre[0] + side_Offset, leftCentre[1])

        img_circle = cv2.circle(img_circle, (centre[0], 200), 5, self.dotColour, -1)
        # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
        img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
        cv2.imshow("Debug Image",img_line)
        cv2.waitKey(3)

        # Use PID control to determine the rotation and speed of the vehicle
        angular = pid(self, kpAngOne, kiAngOne, kdAngOne, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturationAngOne, 'OneAng')
        self.move = (speed, angular)
        print("Following Left line")

      # Follow Right Side
      elif botcentre[0] > int(cols / 2):
        rightCentre = (int((topright[0] + botcentre[0]) / 2), int((topright[1] + botcentre[1]) / 2))

        centre = (rightCentre[0] - side_Offset, rightCentre[1])

        img_circle = cv2.circle(img_circle, (centre[0], 200), 5, self.dotColour, -1)
        # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
        img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
        cv2.imshow("Debug Image",img_line)
        cv2.waitKey(3)

        # Use PID control to determine the rotation and speed of the vehicle
        angular = pid(self, kpAngOne, kiAngOne, kdAngOne, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturationAngOne, 'OneAng')
        self.move = (speed, angular)
        print("Following Right line")

    # There is one contour in the bottom, less than 2 on the top, follow the bottom side
    elif len(botcontours) == 1:
      
      botContMom = cv2.moments(botcontours[0][1])

      # Find and draw centre of biggest bottom contour
      botcentre = (int(botContMom['m10']/botContMom['m00']), int(botContMom['m01']/botContMom['m00']))
      img_circle = cv2.circle(img_circle, (botcentre[0], rows - midcrop_height + botcentre[1]), 5, self.dotColour, -1)

      # Follow Left Side
      if botcentre[0] <= int(cols / 2):
        
        centre = (botcentre[0] + side_Offset, botcentre[1])

        img_circle = cv2.circle(img_circle, (centre[0], 200), 5, self.dotColour, -1)
        # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
        img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
        cv2.imshow("Debug Image",img_line)
        cv2.waitKey(3)

        # Use PID control to determine the rotation and speed of the vehicle
        angular = pid(self, kpAngOne, kiAngOne, kdAngOne, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturationAngOne, 'OneAng')
        self.move = (speed, angular)
        print("Following Left line")

      # Follow Right Side
      elif botcentre[0] > int(cols / 2):
        
        centre = (botcentre[0] - side_Offset, botcentre[1])

        img_circle = cv2.circle(img_circle, (centre[0], 200), 5, self.dotColour, -1)
        # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
        img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
        cv2.imshow("Debug Image",img_line)
        cv2.waitKey(3)

        # Use PID control to determine the rotation and speed of the vehicle
        angular = pid(self, kpAngOne, kiAngOne, kdAngOne, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturationAngOne, 'OneAng')
        self.move = (speed, angular)
        print("Following Right line")


      # maxCont = max(contours, key=cv2.contourArea)
      # contMoments = cv2.moments(maxCont)
      # contCentre = (int(contMoments['m10']/contMoments['m00']), int(contMoments['m01']/contMoments['m00']))

      # if contCentre[0] < int(cols/2):
      #   centre = (contCentre[0] + side_Offset, contCentre[1])
      # elif contCentre[0] > int(cols/2):
      #   centre = (contCentre[0] - side_Offset, contCentre[1])
      # else:
      #   centre = contCentre

      # img_circle = cv2.circle(cv_image, (centre[0], rows - crop_height + centre[1]), 5, (0, 0, 255), -1)
      # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), (0, 0, 255), 1)
      # img_line = cv2.line(img_line, (targetAng, 0), (targetAng, rows), (0, 0, 255), 1)
      # # cv2.imshow("Circle Image",img_line)
      # # cv2.waitKey(3)

      # # Use PID control to determine the rotation and speed of the vehicle
      # angular = pid(self, kpAngOne, kiAngOne, kdAngOne, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturationAngOne, 'OneAng')
      # linear = 0.5
      # self.move = (linear, angular)
      # print("Lost line")

    # if the line is not detected
    # If the camera loses the line then go slowly with the last rotation determined from PID
    else:
      linear = -0.2
      angular = 3*self.LastOutputNormLine
      self.move = (linear, angular)
      print("Lost line")
      cv2.imshow("Debug Image", processed_img)
      print("Top: " + str(len(topcontours)) + "\tBottom: " + str(len(botcontours)))

    
    return state, self.move

  def process_img(self, cv_image):
    # # Variables
    # gaussianKernel = (11, 11)
    # threshold = 220

    # # Convert the frame to a different grayscale
    # img_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    # # Blur image to reduce noise
    # img_blur = cv2.GaussianBlur(img_gray, gaussianKernel, 0)
    # # Binarize the image
    # _, img_bin = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY)
    # # Do more image processing to remove noise
    # img_ero = cv2.erode(img_bin, None, iterations = 2)
    # img_dil = cv2.dilate(img_ero, None, iterations = 2)
    # cv2.imshow("Noise Suppressed", img_dil)
    # cv2.waitKey(3)

    # return img_dil
    
    #Variables
    gaussianKernel = (11, 11)
    threshold = 220
    colourMin = (0, 0, 180)
    colourMax = (180, 50, 255)

    # Get HSV
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Get range of colours
    img_mask = cv2.inRange(img_hsv, colourMin, colourMax)
    # cv2.imshow("HSV Mask", img_mask)
    # cv2.waitKey(3)
    # Blur image 
    img_blur = cv2.GaussianBlur(img_mask, gaussianKernel, 0)
    # Binarize the image
    _, img_bin = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY)
    # Do more image processing to remove noise
    img_ero = cv2.erode(img_bin, None, iterations = 2)
    img_dil = cv2.dilate(img_ero, None, iterations = 2)
    # cv2.imshow("Noise Suppressed", img_dil)
    cv2.waitKey(3)

    return img_dil

  def state_change(self, img):
    state = "drive"
    bin_threshold = 100
    red_threshold = 1500000
    red_img = img[:,:,2] - 0.5 * img[:,:,0] - 0.5 * img[:,:,1]
    _, redbin_img = cv2.threshold(red_img, bin_threshold, 255, cv2.THRESH_BINARY)
    # cv2.imshow("Red Binary", redbin_img)
    # cv2.waitKey(3)
    # print("Red Value: " + str(sum(map(sum, redbin_img))))
    if sum(map(sum, redbin_img)) > red_threshold:
      state = "cross_walk"
    return state

  def sortContours(self, contours):
    areaArray = []
    for _, c in enumerate(contours):
      area = cv2.contourArea(c)
      areaArray.append(area)
    #first sort the array by area
    sortedContours = sorted(zip(areaArray, contours), key=lambda x: x[0], reverse=True)
    return sortedContours
  
  def followTwoLines(self, cols, rows, topleft, topright, botleft, botright, img):
    kpAng = 0.015
    kiAng = 0
    kdAng = 0.00001
    saturationAng = 2
    targetAng = int(cols/2)
    speed = 0.25

    # Find Average of all of the centres
    ave_centre = (int((topleft[0] + botleft[0] + topright[0] + botright[0]) / 4), int((topleft[1] + botleft[1] + topright[1] + botright[1]) / 4))
    img_circle = cv2.circle(img, (ave_centre[0], 200), 5, self.dotColour, -1)
    img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
    # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
    cv2.imshow("Debug Image", img_line)
    cv2.waitKey(3)

    # Use PID control to determine the rotation and speed of the vehicle
    angular = pid(self, kpAng, kiAng, kdAng, targetAng, ave_centre[0], self.errSumNormAng, self.lastErrNormAng, self.lastTimeNormAng, saturationAng, 'NormAng')
    self.move = (speed, angular)

    print("Reguler 2 lines")


  def followOneLine(self, cols, rows, offset, contCentre, img):
    kp = 0.015
    ki = 0
    kd = 0.00001
    saturation = 2
    targetAng = int(cols/2)
    speed = 0.25
    
    if contCentre[0] <= targetAng:
      centre = (contCentre[0] + offset, contCentre[1])
    else:
      centre = (contCentre[0] - offset, contCentre[1])

    img_circle = cv2.circle(img, (centre[0], 200), 5, self.dotColour, -1)
    # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
    img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
    cv2.imshow("Debug Image",img_line)
    cv2.waitKey(3)

    # Use PID control to determine the rotation and speed of the vehicle
    angular = pid(self, kp, ki, kd, targetAng, centre[0], self.errSumOneAng, self.LastErrOneAng, self.LastTimeOneAng, saturation, 'OneAng')
    self.move = (speed, angular)

    print("Following One line")



def main(args):
  rospy.init_node('robot_driver', anonymous=True)
  rd = robot_driver()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
  rospy.signal_shutdown("Keyboard interrupt")

if __name__ == '__main__':
    main(sys.argv)