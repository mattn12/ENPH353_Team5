#!/usr/bin/env python3
from __future__ import print_function

import cv2
import sys
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

import roslib
import rospy

from move_robot import robot_driver
from read_plate import plate_reader
from cross_walk import avoid_ped


## States

class state_machine:

  def __init__(self):

      # Variables for subscribing and publishing
      self.bridge = CvBridge()
      print("Created Bridge")
      self.image_sub = rospy.Subscriber("/R1/pi_camera/image_raw",Image,self.run)
      self.drive_pub = rospy.Publisher("/R1/cmd_vel", Twist, queue_size=1)
      self.license_pub = rospy.Publisher("/license_plate", String, queue_size=1)

      # State Variables
      self.states = {"drive", "cross_walk", "found_car"}
      self.currentState = "drive"
      self.drive = robot_driver()
      self.cross = avoid_ped()
      # self.plate = plate_reader()

      # Other Variables
      self.move = Twist()

      # start the timer
      output = str('Team5,password,0,ABCD')
    
      try:
        self.license_pub.publish(output)
        rospy.sleep(0.5)
        rospy.Timer(rospy.Duration(10),self.stop_robot)
      except CvBridgeError as e:
        print(e)


  def state_change(self, state):
      if self.states.contains(state):
          self.currentState = state
      else:
          print("Not a state")

  def run(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      print("\n\n\n\nGot an Image\n\n\n\n")
    except CvBridgeError as e:
      print(e)

    if self.currentState == "drive":
      print("\n\nDriving\n\n")
      self.currentState, mover = self.drive.run_drive(cv_image)
      # self.move.linear.x = mover[0]
      # self.move.angular.z = mover[1]
      (self.move.linear.x, self.move.angular.z) = mover
      # self.move.linear.x = 0
      # self.move.angular.z = 0
    elif self.currentState == "cross_walk":
      print("\n\nCross Walk\n\n")
      self.currentState, mover =  self.cross.run_cross_walk(cv_image)
      # self.move.linear.x = mover[0]
      # self.move.angular.z = mover[1]
      (self.move.linear.x, self.move.angular.z) = mover
    else:
      print("The given state: %s does not match a known state", self.currentState)


    # Run Matthew's License Plate Code


    try:
      self.drive_pub.publish(self.move)
      print(str('Published: LinX = {}, AngZ = {}').format(self.move.linear.x,self.move.angular.z))

    except CvBridgeError as e:
      print(e)

  def stop_robot(self):
    print("\n\n\n\n\n\n\nSTOP!!!!\n\n\n\n\n\n\n\n")
    output = str('Team5,password,-1,ABCD')
    self.move.angular.z = 0
    self.move.linear.x = 0
    try:
      self.license_pub.publish(output)
      self.drive_pub.publish(self.move)
    except CvBridgeError as e:
      print(e)

    self.endRun = True

    print("Stopped robot")
    rospy.signal_shutdown("Timer ended.")

  # def run_drive(self):
  #   print("\n\n\n\n\n\nRun Drive\n\n\n\n\n\n")
  #   self.ic.callback(data = self.currentState)

  # def run_cross_walk(self):
  #   print("\n\n\n\n\n\nRun Cross Walk\n\n\n\n\n\n")
  #   self.ic.callback()

  # def run_found_car(self):
  #   print("\n\n\n\n\n\nRun Found Car\n\n\n\n\n\n")
  #   self.ic.callback()


def main(args):
  

  print("Entered Main")
  rospy.init_node('state_machine', anonymous=True)
  print("Created State Machine node")
  sm = state_machine()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)