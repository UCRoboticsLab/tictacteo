#!/usr/bin/python

#####################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to use the Baxter robot infra-red 
# sensor to detect the location of the tig tag toe
# game playing table
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
from copy import deepcopy

import matplotlib.pylab as plt
import matplotlib.cm as cm
import rospy
import baxter_interface
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range

import math

class GameTableFinder(object):

    def __init__(self):
        
        self.__ready = False
        
        rospy.init_node('tablefinder',anonymous = True)
        self.__left_ir_sensor  = rospy.Subscriber('/robot/range/left_hand_range/state' ,Range, callback=self.__ir_sensor_callback, callback_args="left",queue_size=1)
        self.__right_ir_sensor = rospy.Subscriber('/robot/range/right_hand_range/state' ,Range, callback=self.__ir_sensor_callback, callback_args="right",queue_size=1)
        
        
    
    
    
    def __ir_sensor_callback(self, msg, side):
        
        print side, msg
    
    
    def is_ready():
        
        return self.__ready
        
    def find_edge(self):
    
        pass


    def run(self):
    
        while not rospy.is_shutdown():
        
        
        
            rospy.sleep(0.1)
        
    
    
    




def main():

    gtf = GameTableFinder()
    gtf.run()



if __name__ == '__main__':
    sys.exit(main())
