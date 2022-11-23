#!/usr/bin/env python3
from __future__ import print_function

import cv2
import sys
import numpy as np

import roslib
import rospy

from move_robot import robot_driver
from read_plate import plate_reader


## States

class state_machine:

  def __init__(self):
      self.states = {"drive", "cross_walk", "found_car"}
      self.currentState = "drive"


  def state_change(self, state):
      if self.states.contains(state):
          self.currentState = state
      else:
          print("Not a state")

  def run(self):
      while True:
        if self.currentState == "drive":
          self.run_drive(self)
        elif self.currentState == "cross_walk":
          self.run_cross_walk(self)
        elif self.currentState == "found_car":
          self.run_found_car(self)
        else:
          print("The given state: %s does not match a known state", self.currentState)

  def run_drive(self):
    print("\n\n\n\n\n\nRun Drive\n\n\n\n\n\n")
    self.ic.callback(data = self.currentState)

  def run_cross_walk(self):
    print("\n\n\n\n\n\nRun Cross Walk\n\n\n\n\n\n")
    self.ic.callback()

  def run_found_car(self):
    print("\n\n\n\n\n\nRun Found Car\n\n\n\n\n\n")
    self.ic.callback()


def main(args):
  
  rospy.init_node('state_machine', anonymous=True)
  sm = robot_driver()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)