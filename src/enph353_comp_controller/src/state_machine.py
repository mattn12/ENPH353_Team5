#!/usr/bin/env python3
from __future__ import print_function

import cv2
import sys
import numpy as np

import roslib
import rospy

from move_robot import robot_driver
from read_plate import plate_reader

class state_machine:

    def __init__(self):
        self.states = {"drive", "cross_walk", "found_car"}
        self.currentState

    def state_change(self, state):
        if self.states.contains(state):
            self.currentState = state
        else:
            print("Not a state")

    # def run(self):
    #     while True:

def main(args):
  
  rospy.init_node('plate_reader', anonymous=True)
  rd = robot_driver()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)