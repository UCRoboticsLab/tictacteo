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
from std_msgs.msg import String
import message_filters

import math

class GameTableFinder(object):

    def __init__(self, table_width, table_length, table_height):
        
        self.__ready = False
        
        self.table_width = table_width
        self.table_length = table_length
        self.table_height = table_height
        
        self.rb_cmd_pub = rospy.Publisher('robot_arms_cmd', String, queue_size=10)
        #self._left_ir_sensor  = rospy.Subscriber('/robot/range/left_hand_range/state' ,Range, callback=self._ir_sensor_callback, callback_args="left",queue_size=1)
        #self._right_ir_sensor = rospy.Subscriber('/robot/range/right_hand_range/state' ,Range, callback=self._ir_sensor_callback, callback_args="right",queue_size=1)
        left_ir_msg = message_filters.Subscriber('/robot/range/left_hand_range/state',Range)
        right_ir_msg = message_filters.Subscriber('/robot/range/right_hand_range/state',Range)
        ts = message_filters.ApproximateTimeSynchronizer([left_ir_msg, right_ir_msg], 10, 0.05)
        ts.registerCallback(self._ir_sensor_callback)
        
        self.current_ir_readings = None
        
    
    
    
    def _ir_sensor_callback(self, left_msg, right_msg): #def _ir_sensor_callback(self, msg, side):
        
        print "\nLeft IR: \n", left_msg.range, "\nRight IR: \n", right_msg.range
        return
    
    
    
    
    def is_ready():
        
        return self.__ready
        
    def find_edge(self):
    
        pass


    




def main():

    gtf = GameTableFinder(1.0, 0.5, 0.9)
    rospy.init_node('tablefinder',anonymous = True)
    
    while not rospy.is_shutdown():
        
        
        
        rospy.sleep(0.1)



if __name__ == '__main__':
    sys.exit(main())
