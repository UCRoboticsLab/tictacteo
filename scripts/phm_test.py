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
from copy import deepcopy

import matplotlib.pylab as plt
import matplotlib.cm as cm

import baxter_interface
from sensor_msgs.msg import Image


class TigTagToeGame(object):

    def __init__(self):
    
        self.cameras = [
    
    
    def game_init(self):
    
    
    def game_calibration(self):
    
    
    def camera_callback(self):
    



def main():



if __name__ == '__main__':
    sys.exit(main())
