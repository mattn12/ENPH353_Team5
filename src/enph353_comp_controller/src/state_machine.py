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
from inner_road import inner_road_driver

from time import sleep


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
    self.states = {"drive", "cross_walk", "found_car", "turn_left", "inner_road"}
    self.currentState = "turn_left"
    self.published_plate = False
    self.plate_pic = np.empty(0)
    self.position_pic = np.empty(0)

    # output to start the timer
    self.output = str('Team5,password,0,ABCD')

    # Initialize Classes
    self.drive = robot_driver()
    self.cross = avoid_ped()
    self.plate_read = plate_reader()
    self.inner_drive = inner_road_driver()
    self.move = Twist()
    print("Initialized Variables")

    try:
      self.license_pub.publish(self.output)
      rospy.Timer(rospy.Duration(160),self.stop_robot)
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
      print("\n\nGot an Image\n\n")
    except CvBridgeError as e:
      print(e)

    if self.currentState == "drive":
      print("\nDriving\n")
      self.currentState, mover, self.plate_pic, self.position_pic = self.drive.run_drive(cv_image,self.published_plate)
      self.published_plate = False
      (self.move.linear.x, self.move.angular.z) = mover

    elif self.currentState == "cross_walk":
      print("\nCross Walk\n")
      self.currentState, mover =  self.cross.run_cross_walk(cv_image)
      (self.move.linear.x, self.move.angular.z) = mover

    elif self.currentState == "inner_road":
      print("\nInner Road\n")
      print(self.inner_drive)
      self.currentState, mover, self.plate_pic, self.position_pic = self.inner_drive.run_inner_drive(cv_image, self.published_plate)
      self.published_plate = False
      (self.move.linear.x, self.move.angular.z) = mover

    elif self.currentState == "found_car":
      print("\nReading Plate\n")
      mover = (0.1,0)
      self.currentState, self.published_plate, self.output =  self.plate_read.read_plate(self.plate_pic, self.position_pic)
      (self.move.linear.x, self.move.angular.z) = mover
      if self.output != "ERROR":
        try:
          print(self.output)
          self.license_pub.publish(self.output)
          rospy.sleep(0.2)
        except CvBridgeError as e:
          print(e)
      else:
        self.currentState = "drive"
    
    elif self.currentState == "turn_left":
      self.currentState = "drive"
      (self.move.linear.x, self.move.angular.z) = (0.3, 0.95)
      sleep(3)

    else:
      print("The given state: %s does not match a known state", self.currentState)


    try:
      self.drive_pub.publish(self.move)
      print(str('Published: LinX = {}, AngZ = {}').format(self.move.linear.x,self.move.angular.z))

    except CvBridgeError as e:
      print(e)

  def stop_robot(self, event):
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