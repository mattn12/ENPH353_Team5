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
    self.dotColour = (0, 0, 255)



  def run_drive(self, cv_image):
    # Constants
    state = "drive"
    (rows,cols,channels) = cv_image.shape
    topcrop_height = 200
    midcrop_height = 100
    side_Offset = 400
    # Speed(forward/back, angular)
    self.move = (0.0, 0.0)

    ## Check the state
    state = self.state_change(cv_image)
    if state != "drive":
      return state, (0.0,0.0)

    ## Image Processing

    processed_img = self.process_img(cv_image)
    #todo Determine if we need to change state


    # processed_img = cv2.cvtColor(processed_img, cv2.COLOR_GRAY2RGB)

    # processed_img = cv2.circle(processed_img, (0, 0), 50, (255, 0, 0), -1)
    # processed_img = cv2.circle(processed_img, (0, rows), 50, (0, 255, 0), -1)
    # processed_img = cv2.circle(processed_img, (cols, rows), 50, (0, 0, 255), -1)
    # processed_img = cv2.circle(processed_img, (cols, 0), 50, (100, 100, 100), -1)


    # slice bottom portion of image
    img_top = processed_img[rows - topcrop_height:rows - midcrop_height,:]
    # cv2.imshow("TopCropped", img_top)
    # cv2.waitKey(3)
    img_bot = processed_img[rows - midcrop_height:,:]
    # cv2.imshow("BottomCropped", img_bot)
    # cv2.waitKey(3)

    # Find the contours of the image
    topcontours, _ = cv2.findContours(img_top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE, offset=(0, rows - topcrop_height))
    botcontours, _ = cv2.findContours(img_bot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE, offset=(0, rows - midcrop_height))




    # If there are at least 2 contours in top and bottom, line follow them
    if len(topcontours) > 1 and len(botcontours) > 1:

      # first sort the array by area
      topSortedCont = self.sortContours(topcontours)
      botSortedCont = self.sortContours(botcontours)

      # Get the moments of the largest contours
      topContMom1 = cv2.moments(topSortedCont[0])
      topContMom2 = cv2.moments(topSortedCont[1])
      botContMom1 = cv2.moments(botSortedCont[0])
      botContMom2 = cv2.moments(botSortedCont[1])

      # Draw Contours on Image
      img_cont1 = cv2.drawContours(cv_image, topSortedCont, 0, (255, 0, 0), 2)
      img_cont2 = cv2.drawContours(img_cont1, topSortedCont, 1, (0, 255, 0), 2)
      img_cont3 = cv2.drawContours(img_cont2, botSortedCont, 0, (255, 0, 0), 2)
      img_cont4 = cv2.drawContours(img_cont3, botSortedCont, 1, (0, 255, 0), 2)

      # # Draw contours on image
      # printCont1 = topSortedCont[0] + (0, topcrop_height)
      # printCont2 = topSortedCont[1] + (0, topcrop_height)
      # printCont3 = botSortedCont[0] + (0, midcrop_height)
      # printCont4 = botSortedCont[1] + (0, midcrop_height)
      
      # img_cont1 = cv2.drawContours(processed_img, printCont1, 0, (100, 100, 100), 10)
      # img_cont2 = cv2.drawContours(img_cont1, printCont2, 0, (100, 100, 100), 10)
      # img_cont3 = cv2.drawContours(img_cont2, printCont3, 0, (100, 100, 100), 10)
      # img_cont4 = cv2.drawContours(img_cont3, printCont4, 0, (100, 100, 100), 10)

      # For troubleshooting
      # print("Contour Moment Areas:\n" + str(topContMom1['m00']) + "\n" + str(topContMom2['m00']) + "\n" + str(botContMom1['m00']) + "\n" + str(botContMom2['m00']) + "\n")

      # Check if the contour contains any pixels
      if topContMom1['m00'] > 0 and topContMom2['m00'] > 0 and botContMom1['m00'] > 0 and botContMom2['m00'] > 0:
        # Find and draw centre of biggest top contour
        topcentre1 = (int(topContMom1['m10']/topContMom1['m00']), int(topContMom1['m01']/topContMom1['m00']))
        img_circle = cv2.circle(img_cont4, (topcentre1[0], topcentre1[1]), 5, self.dotColour, -1)

        # Find and draw centre of second biggest top contour
        topcentre2 = (int(topContMom2['m10']/topContMom2['m00']), int(topContMom2['m01']/topContMom2['m00']))
        img_circle = cv2.circle(img_circle, (topcentre2[0], topcentre2[1]), 5, self.dotColour, -1)
        
        # Find and draw centre of biggest bottom contour
        botcentre1 = (int(botContMom1['m10']/botContMom1['m00']), int(botContMom1['m01']/botContMom1['m00']))
        img_circle = cv2.circle(img_circle, (botcentre1[0], botcentre1[1]), 5, self.dotColour, -1)

        # Find and draw centre of biggest bottom contour
        botcentre2 = (int(botContMom2['m10']/botContMom2['m00']), int(botContMom2['m01']/botContMom2['m00']))
        img_circle = cv2.circle(img_circle, (botcentre2[0], botcentre2[1]), 5, self.dotColour, -1)

        if topcentre1[0] < topcentre2[0]:
          topleft, topright = topcentre1, topcentre2
        else:
          topleft, topright = topcentre2, topcentre1

        if botcentre1[0] < botcentre2[0]:
          botleft, botright = botcentre1, botcentre2
        else:
          botleft, botright = botcentre2, botcentre1

        # Follow Centre of two lines
        if topleft[0] > botleft[0] and topright[0] < botright[0]:

          self.followTwoLines(cols, rows, topleft, topright, botleft, botright, img_circle)

        # Follow right line
        elif topleft[0] < botleft[0] and topright[0] < botright[0]:
          
          rightCentre = (int((topright[0] + botright[0]) / 2), int((topright[1] + botright[1]) / 2))

          self.followOneLine(cols, rows, side_Offset, rightCentre, img_circle)

        # Follow left line
        elif topleft[0] > botleft[0] and topright[0] > botright[0]:
          
          leftCentre = (int((topleft[0] + botleft[0]) / 2), int((topleft[1] + botleft[1]) / 2))

          self.followOneLine(cols, rows, side_Offset, leftCentre, img_circle)

      # The area of a contour is zero
      else:
        img_text = cv2.putText(cv_image, "Zero Area", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Debug Image", img_text)
        cv2.waitKey(3)




    # There is one contour in the top, follow the side with the top contour
    elif len(topcontours) == 1 and len(botcontours) > 1:
      
      #Sort Bottom Contours
      botSortedCont = self.sortContours(botcontours)

      #Get Largest Contours
      topContMom = cv2.moments(topcontours[0])
      botContMom1 = cv2.moments(botSortedCont[0])
      botContMom2 = cv2.moments(botSortedCont[1])

      # Draw contours on image
      img_cont1 = cv2.drawContours(cv_image, topcontours, -1, (255, 0, 0), 2)
      img_cont2 = cv2.drawContours(img_cont1, botSortedCont, 0, (255, 0, 0), 2)
      img_cont3 = cv2.drawContours(img_cont2, botSortedCont, 1, (0, 255, 0), 2)

      # Check if the contour contains any pixels
      if topContMom['m00'] > 0 and botContMom1['m00'] > 0 and botContMom2['m00'] > 0:
        # Get Centroid of contours
        topcentre = (int(topContMom['m10']/topContMom['m00']), int(topContMom['m01']/topContMom['m00']))
        botcentre1 = (int(botContMom1['m10']/botContMom1['m00']), int(botContMom1['m01']/botContMom1['m00']))
        botcentre2 = (int(botContMom2['m10']/botContMom2['m00']), int(botContMom2['m01']/botContMom2['m00']))

        #Classify which side the bottom contours are on
        if botcentre1[0] < botcentre2[0]:
          botleft, botright = botcentre1, botcentre2
        else:
          botleft, botright = botcentre2, botcentre1

        # If the top line is on the left, follow the left side
        if topcentre[0] <= int(cols / 2):

          leftCentre = (int((topcentre[0] + botleft[0]) / 2), int((topcentre[1] + botleft[1]) / 2))
          img_circle = cv2.circle(img_cont3, (leftCentre[0], leftCentre[1]), 5, self.dotColour, -1)
          
          self.followOneLine(cols, rows, side_Offset, leftCentre, img_circle)

        # Else, follow the right side
        else:
          rightCentre = (int((topcentre[0] + botright[0]) / 2), int((topcentre[1] + botright[1]) / 2))
          img_circle = cv2.circle(img_cont3, (rightCentre[0], rightCentre[1]), 5, self.dotColour, -1)

          self.followOneLine(cols, rows, side_Offset, rightCentre, img_circle)
      
      else:
        img_text = cv2.putText(cv_image, "Zero Area", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Debug Image", img_text)
        cv2.waitKey(3)




    # There is one contour in the bottom, follow the side with the bottom contour
    elif len(topcontours) > 1 and len(botcontours) == 1:

      # Sort the top contours
      topSortedCont = self.sortContours(topcontours)

      # Select the largest contours
      topContMom1 = cv2.moments(topSortedCont[0])
      topContMom2 = cv2.moments(topSortedCont[1])
      botContMom = cv2.moments(botcontours[0])

      # Draw contours on image
      img_cont1 = cv2.drawContours(cv_image, topSortedCont, 0, (255, 0, 0), 2)
      img_cont2 = cv2.drawContours(img_cont1, topSortedCont, 1, (0, 255, 0), 2)
      img_cont3 = cv2.drawContours(img_cont2, botcontours, -1, (255, 0, 0), 2)

      # Check if the contour contains any pixels
      if topContMom1['m00'] > 0 and topContMom2['m00'] > 0 and botContMom['m00'] > 0:
        # Get the centroids of the contours
        topcentre1 = (int(topContMom1['m10']/topContMom1['m00']), int(topContMom1['m01']/topContMom1['m00']))
        topcentre2 = (int(topContMom2['m10']/topContMom2['m00']), int(topContMom2['m01']/topContMom2['m00']))
        botcentre = (int(botContMom['m10']/botContMom['m00']), int(botContMom['m01']/botContMom['m00']))

        # Classify the top centroids based on which side they are on
        if topcentre1[0] < topcentre2[0]:
          topleft, topright = topcentre1, topcentre2
        else:
          topleft, topright = topcentre2, topcentre1

        # If the bottom centroid is on the left, follow the left side
        if botcentre[0] <= int(cols / 2):
          #Get the centroid of the two left centroids
          leftCentre = (int((topleft[0] + botcentre[0]) / 2), int((topleft[1] + botcentre[1]) / 2))
          img_circle = cv2.circle(img_cont3, (leftCentre[0], leftCentre[1]), 5, self.dotColour, -1)
          self.followOneLine(cols, rows, side_Offset, leftCentre, img_circle)

        # Follow the right side
        else:
          rightCentre = (int((topright[0] + botcentre[0]) / 2), int((topright[1] + botcentre[1]) / 2))
          img_circle = cv2.circle(img_cont3, (rightCentre[0], rightCentre[1]), 5, self.dotColour, -1)
          self.followOneLine(cols, rows, side_Offset, rightCentre, img_circle)

      else:
        img_text = cv2.putText(cv_image, "Zero Area", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Debug Image", img_text)
        cv2.waitKey(3)




    # There is one contour in the bottom, less than 2 on the top, follow the bottom side
    elif len(botcontours) == 1:
      
      #Get Bottom contour
      botContMom = cv2.moments(botcontours[0])

      img_cont = cv2.drawContours(cv_image, botcontours, 0, (255, 0, 0), 2)

      # Check if the contour contains any pixels
      if botContMom['m00'] > 0:
        # Get the centroid of the contour
        botcentre = (int(botContMom['m10']/botContMom['m00']), int(botContMom['m01']/botContMom['m00']))
        img_circle = cv2.circle(img_cont, (botcentre[0], botcentre[1]), 5, self.dotColour, -1)

        # Follow the line
        self.followOneLine(cols, rows, side_Offset, botcentre, img_circle)

      else:
        img_text = cv2.putText(cv_image, "Zero Area", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow("Debug Image", img_text)
        cv2.waitKey(3)




    # if the line is not detected
    # If the camera loses the line then go slowly with the last rotation determined from PID
    else:
      linear = -0.2
      angular = 3*self.LastOutputNormLine
      self.move = (linear, angular)
      print("Lost line")
      img_text = cv2.putText(cv_image, "Lost Line", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
      cv2.imshow("Debug Image", img_text)
      cv2.waitKey(3)
      print("Top: " + str(len(topcontours)) + "\tBottom: " + str(len(botcontours)))

    
    return state, self.move

  def process_img(self, cv_image):
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
    
    sortedContours = sorted(contours, key=cv2.contourArea, reverse=True)

    # For Debugging
    # print("Contours by area:\n")
    # for cont in sortedContours:
    #   print(str(cv2.contourArea(cont)) + "\n")

    return sortedContours
  
  def followTwoLines(self, cols, rows, topleft, topright, botleft, botright, img):
    kp = 0.01
    ki = 0
    kd = 0.0002
    saturation = 2
    target = int(cols/2)
    speed = 0.4

    # Find Average of all of the centres
    ave_centre = (int((topleft[0] + botleft[0] + topright[0] + botright[0]) / 4), int((topleft[1] + botleft[1] + topright[1] + botright[1]) / 4))
    img_circle = cv2.circle(img, (ave_centre[0], 200), 10, self.dotColour, -1)
    img_line = cv2.line(img_circle, (target, 0), (target, rows), self.dotColour, 1)
    # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
    img_text = cv2.putText(img_line, "Two Lines", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow("Debug Image", img_text)
    cv2.waitKey(3)

    # Use PID control to determine the rotation and speed of the vehicle
    angular = pid(self, kp, ki, kd, target, ave_centre[0], self.errSumNormAng, self.lastErrNormAng, self.lastTimeNormAng, saturation, 'NormAng')
    self.move = (speed, angular)

    print("Reguler 2 lines")


  def followOneLine(self, cols, rows, offset, contCentre, img):
    kp = 0.01
    ki = 0
    kd = 0.0001
    saturation = 2
    targetAng = int(cols/2)
    speed = 0.4
    
    if contCentre[0] <= targetAng:
      centre = (contCentre[0] + offset, contCentre[1])
      text = "One Line:\tLeft"
    else:
      centre = (contCentre[0] - offset, contCentre[1])
      text = "One Line:\tRight"

    img_circle = cv2.circle(img, (centre[0], 200), 10, self.dotColour, -1)
    # img_line = cv2.line(img_circle, (0, targetLine), (cols, targetLine), self.dotColour, 1)
    img_line = cv2.line(img_circle, (targetAng, 0), (targetAng, rows), self.dotColour, 1)
    img_text = cv2.putText(img_line, text, (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow("Debug Image", img_text)
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