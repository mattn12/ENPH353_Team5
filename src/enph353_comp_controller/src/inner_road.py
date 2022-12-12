#!/usr/bin/env python3
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import time
import numpy as np
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

class inner_road_driver:

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

    self.plate_timerStart = 0

    # Timing variables
    # self.startRun = True
    # self.endRun = False

    # Other Variables
    self.move = (0.0, 0.0)
    # self.lastmove = (0.0, 0.0)
    self.dotColour = (0, 0, 255)

  # To test cross walk
  # def run_drive(self, cv_image):
  #   state = self.state_change(cv_image)
  #   if state != "drive":
  #     return state, (0.0,0.0)
  #   return  state, (0.5, 0)

  def run_inner_drive(self, cv_image, published_plate):
    # Constants
    state = "inner_road"
    (rows,cols,_) = cv_image.shape
    crop = 400
    side_Offset = 419
    right_side_offset = 500
    
    # Speed(forward/back, angular)
    self.move = (0.0, 0.0)
    
    # Assume no plate/position in view
    plate = np.empty(0)
    position = np.empty(0)


    ## Check the state
    state, plate, position = self.state_change(np.copy(cv_image), published_plate)
    if state == "cross_walk":
      return state, (0.0,0.0), plate, position
    elif state == "found_car":
      return state, (-0.05,0.0), plate, position
    

    ## Image Processing for driving

    processed_img = self.process_img(cv_image)
    processed_img = cv2.dilate(processed_img, np.ones((3,3)), iterations = 1)
    processed_img = processed_img()

    
    # cv2.namedWindow("Processed Line Following",cv2.WINDOW_NORMAL)
    # cv2.imshow("Processed Line Following", processed_img)
    # cv2.waitKey(3)
    #todo Determine if we need to change state


    # processed_img = cv2.cvtColor(processed_img, cv2.COLOR_GRAY2RGB)

    # processed_img = cv2.circle(processed_img, (0, 0), 50, (255, 0, 0), -1)
    # processed_img = cv2.circle(processed_img, (0, rows), 50, (0, 255, 0), -1)
    # processed_img = cv2.circle(processed_img, (cols, rows), 50, (0, 0, 255), -1)
    # processed_img = cv2.circle(processed_img, (cols, 0), 50, (100, 100, 100), -1)



    # Find the contours of the image
    contours, _ = cv2.findContours(processed_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    img_text = cv_image

    cv2.namedWindow("Debug Image", cv2.WINDOW_NORMAL)

    # There is at least one contour, follow it
    if len(contours) >= 1:
      
      # Sort Contours
      sorted_conts = self.sortContours(contours)

      #Get Bottom contour
      contMom = cv2.moments(sorted_conts[0])

      img_cont = cv2.drawContours(cv_image, sorted_conts, 0, (255, 0, 0), 2)

      # Check if the contour contains any pixels
      if contMom['m00'] > 0:
        # Get the centroid of the contour
        centre = (int(contMom['m10']/contMom['m00']), int(contMom['m01']/contMom['m00']))
        img_circle = cv2.circle(img_cont, (centre[0], centre[1]), 5, self.dotColour, -1)

        # if centre[0] <= int(cols/2):
        #   offset = side_Offset
        # else:
        #   offset =  right_side_offset
  


        # Follow the line
        self.followOneLine(cols, rows, 0, centre, img_circle)

      else:
        img_text = cv2.putText(cv_image, "Zero Area", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        




    # if the line is not detected
    # If the camera loses the line then go slowly with the last rotation determined from PID
    else:
      linear = -0.15
      angular = 2.5*self.LastOutputNormLine
      self.move = (linear, angular)
      print("Lost line")
      img_text = cv2.putText(cv_image, "Lost Line", (int(cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    
    #   print("Top: " + str(len(topcontours)) + "\tBottom: " + str(len(botcontours)))


    cv2.imshow("Debug Image", img_text)
    cv2.waitKey(3)
    return state, self.move, plate, position

  def process_img(self, cv_image):
    #Variables
    gaussianKernel = (11, 11)
    threshold = 220
    colourMin = (80, 125, 125)
    colourMax = (95, 135, 135)

    # Get LAB
    img_lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)

    # Blur image 
    img_blur = cv2.GaussianBlur(img_lab, gaussianKernel, 0)
  
    # Get range of colours
    img_mask = cv2.inRange(img_blur, colourMin, colourMax)
    # cv2.imshow("LAB Mask", img_mask)
    # cv2.waitKey(3)

    # lab_filter = cv2.inRange(cv2.cvtColor(cv_image,cv2.COLOR_BGR2LAB),(120,125,125),(255,135,135))
    # cv2.imshow("LAB", img_lab)
    # cv2.waitKey(3)
    # cv2.imshow("LAB_filter", lab_filter)
    # cv2.waitKey(3)
    # Binarize the image
    # _, img_bin = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY)
    # Do more image processing to remove noise
    # img_ero = cv2.erode(img_mask, None, iterations = 2)
    # img_dil = cv2.dilate(img_ero, None, iterations = 2)
    # cv2.imshow("Noise Suppressed", img_dil)
    # cv2.waitKey(3)


    cv2.imshow("LAB", img_lab)
    cv2.waitKey(3)
    cv2.imshow("Mask", img_mask)
    cv2.waitKey(3)

    return img_mask

  def process_HSV_range(self, img, lower, upper):
    # input:  img: cv2 image
    #         lower: lower bound of HSV values (np array)
    #         upper: upper bound of HSV values (np array)
    # output: filtered cv2 binary image between
    #         lower and upper bounds in HSV colorspace

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # cv2.imshow("hsv",hsv)
    # cv2.waitKey(3)
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
    return cv2.inRange(blurred, lower, upper)

  def process_LAB_range(self, img, lower, upper):
    # input:  img: cv2 image
    #         lower: lower bound of LAB values (np array)
    #         upper: upper bound of LAB values (np array)
    # output: filtered cv2 binary image between
    #         lower and upper bounds in HSV colorspace

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
    return cv2.inRange(blurred, lower, upper)

  def state_change(self, img, published_plate):
    state = "inner_road"
    rows,cols = img.shape[:2]
    cv_image = np.copy(img)

    
    if published_plate:
      print("Started Timer")
      self.plate_timerStart = time.time()
      
    if (time.time() - self.plate_timerStart > 2) and not(state=="cross_walk"):
      # Constants
      img_height = 400
      img_crop = cv_image[rows - img_height:, :]
        
      def find_points(contour):
        # input:    contours of a shape
        # referenced: https://stackoverflow.com/questions/41879315/opencv-visualize-polygonal-curves-extracted-with-cv2-approxpolydp
        # define main island contour approx. and hull
        perimeter = cv2.arcLength(contour, True)
        epsilon = 0.01*perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)
        hull = cv2.convexHull(contour)

        return approx

      def sort_points(points):
        # input:    single entry list of a single entry list of (x,y) tuple points
        # output:   same format of sorted (x,y) points according to:
        #           order: (top left, top right, bottom left, bottom right)

        # Assume the rectangle is not super distorted
        # Then the sum will ensure the placement of the top left and bottom right corners
        sort = sorted(points, key=lambda x: x[0, 0]+x[0, 1])
        # print(sort)

        # check if middle values have been misplaced
        if (sort[1][0][0] < sort[2][0][0]) or (sort[1][0][1] > sort[2][0][1]):
          sort[1], sort[2] = sort[2], sort[1]

        return sort

      def correct_distortion(img, corners, shape):
        # input:    cv2 image
        #           ordered corners of a distorted rectangle
        #           shape of output image (height,width)
        # output:   cv2 image of desired shape

        # Order of corners: top left, top right, bottom left, bottom right)

        # referenced https://arccoder.medium.com/straighten-an-image-of-a-page-using-opencv-313182404b06
        dstPts = [[0, 0], [shape[1], 0], [0, shape[0]], [shape[1], shape[0]]]
        m = cv2.getPerspectiveTransform(np.float32(corners), np.float32(dstPts))

        # Transform the image
        return cv2.warpPerspective(img, m, (int(shape[1]), int(shape[0])))

      ###################### FINDING THE PLATE ################################
      plate_lower = np.array([80, 0, 75])
      plate_upper = np.array([150, 65, 180])
      plate_mask = self.process_HSV_range(img_crop,plate_lower,plate_upper)
      
      plate_contours, _ = cv2.findContours(plate_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) 
      ###################### FINDING THE PLATE ################################



      ###################### FINDING THE POSITION NUMBER ################################
      lower_forPos = np.array([0, 0, 90])
      upper_forPos = np.array([0, 0, 210])
      mask_forPos = self.process_HSV_range(img_crop, lower_forPos, upper_forPos)

      contours_Pos, _ = cv2.findContours(mask_forPos, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
      ###################### FINDING THE POSITION NUMBER ################################


      if len(plate_contours) > 0:
        
        ###################### FINDING THE PLATE ################################
        #first sort the array by area
        sortedCont_Plate = sorted(plate_contours, key=cv2.contourArea, reverse=True)
        cnt = sortedCont_Plate[0]
        check_area = cv2.contourArea(cnt)
        points = find_points(cnt)
        ###################### FINDING THE PLATE ################################

        ###################### FINDING THE POSITION NUMBER ################################
        sortedCont_Pos = sorted(contours_Pos, key=cv2.contourArea, reverse=True)
        cnt_Pos = sortedCont_Pos[0]
        check_areaPos = cv2.contourArea(cnt_Pos)
        points_Pos = find_points(cnt_Pos)
        ###################### FINDING THE POSITION NUMBER ################################

        ##### Draw outlines and the corners with respective areas #########################
        img_draw = np.copy(img_crop)
        img_draw = cv2.putText(img_draw, str(check_area), (int(
            cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        img_draw = cv2.putText(img_draw, str(check_areaPos), (int(
            cols / 2), 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.drawContours(img_draw, [cnt], -1, (255, 0, 0), 3)
        cv2.drawContours(img_draw, points, -1, (0, 0, 255), 3)
        cv2.drawContours(img_draw, [cnt_Pos], -1, (255, 0, 0), 3)
        cv2.drawContours(img_draw, points_Pos, -1, (0, 0, 255), 3)
        cv2.namedWindow("Plate Finding",cv2.WINDOW_NORMAL)
        cv2.imshow("Plate Finding",img_draw)
        cv2.waitKey(3)
        ##### Draw outlines and the corners with respective areas #########################



        # If we are close enough, found the car (area big enough to avoid noise)
        # Get and trasform the plate and position number, send to read_plate
        if (len(points) == 4 and check_area > 3200) and (len(points_Pos) == 4 and check_areaPos > 13500):
          print("Car detected")
          state = "found_car"

          # if there are four corners in each do the perspective trannsform
          img_draw = cv2.putText(img_draw, str(check_area)+" I see a plate", (int(
              cols / 2), 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
          img_draw = cv2.putText(img_draw, str(check_areaPos)+" I see a car", (int(
              cols / 2), 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
          cv2.imshow("Plate Finding", img_draw)
          cv2.waitKey(3)


          sortedPoints = sort_points(points)
          sortedPoints_Pos = sort_points(points_Pos)
          

          ###################### FINDING THE POSITION NUMBER ################################
          norm = correct_distortion(img_crop, sortedPoints_Pos, (230, 230))
          # cut out the position number
          position = norm[120:220, 120:240]

          # cv2.imshow("norm",norm)
          # cv2.waitKey(3)
          ###################### FINDING THE POSITION NUMBER ################################


          ###################### FINDING THE PLATE ################################
          plate = correct_distortion(img_crop, sortedPoints, (50, 230))
          ###################### FINDING THE PLATE ################################


          # cv2.imshow("image5", norm)
          # cv2.waitKey(3)
          # cv2.imshow("image6", position)
          # cv2.waitKey(3) 
          # cv2.namedWindow("Plate and Position", cv2.WINDOW_NORMAL)
          # cv2.imshow("Plate and Position", np.concatenate((np.pad(plate, [(0, 50), (0, 0), (0, 0)], mode='constant'),
          #                                                  position), axis=1))
          # cv2.waitKey(3)
          cv2.namedWindow("Plate", cv2.WINDOW_NORMAL)
          cv2.imshow("Plate", plate)
          cv2.waitKey(3)

          return state,plate,position
    
    # cv2.namedWindow("Red Image", cv2.WINDOW_NORMAL)
    # cv2.imshow("Red Image", img)
    # cv2.waitKey(3)
    print("State: " + state)
    return state, np.empty(0), np.empty(0)

  def sortContours(self, contours):
    
    sortedContours = sorted(contours, key=cv2.contourArea, reverse=True)

    # For Debugging
    # print("Contours by area:\n")
    # for cont in sortedContours:
    #   print(str(cv2.contourArea(cont)) + "\n")

    return sortedContours
  
  def followTwoLines(self, cols, rows, topleft, topright, botleft, botright, img):
    kp = 0.008
    ki = 0
    kd = 0.00002
    saturation = 3
    target = int(cols/2)
    speed = 0.25

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
    kp = 0.015
    ki = 0
    kd = 0.00005
    saturation = 5
    targetAng = int(cols/2)
    speed = 0.15
    
    if contCentre[0] <= targetAng:
      centre = (contCentre[0] + offset, contCentre[1])
      text = "One Line: Left"
    else:
      centre = (contCentre[0] - offset, contCentre[1])
      text = "One Line: Right"

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
  rd = inner_road_driver()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
  rospy.signal_shutdown("Keyboard interrupt")

if __name__ == '__main__':
    main(sys.argv)