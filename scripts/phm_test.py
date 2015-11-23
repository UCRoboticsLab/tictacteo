#!/usr/bin/python

#####################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to use the Baxter robot to play Tig Tag
# Toe Game
# 
#
# Note:
#       1. Before running this program, robot left
#          arm shall be manully put to a proper  
#          position (close to the middle of the table
#          10-30cm above the table)        
#####################################################

import cv2
import threading
import numpy as np
import os
import sys
from cv_bridge import CvBridge
from copy import deepcopy

import matplotlib.pylab as plt
import matplotlib.cm as cm

import math

import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from baxter_core_msgs.msg import EndpointState
import message_filters
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import String
import rospy
import signal
from baxterbase import BaxterBase

class TestBaxterBase(BaxterBase):

    def __init__(self):
        
        
        BaxterBase.__init__(self)
    
    
    def get_img(self, side):
        
        img = None
        if side == 'left':
            
            img = deepcopy(self.left_cur_img)
            
        elif side == 'right':
            
            img = deepcopy(self.right_cur_img)
        
        return img

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)

def main():
    
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("baxter_test")
    
    t1 = TestBaxterBase()
    t1.init_camera()
    
    
    
    while not rospy.is_shutdown():
            
        img = t1.get_img('left')
        if img != None:
            cv2.imshow('current_image', img)
            cv2.waitKey(5)
        rospy.sleep(0.1)


if __name__ == '__main__':
    sys.exit(main())
