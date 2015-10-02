#!/usr/bin/python

#####################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to use the Baxter robot left arm camera
# to find tig tag toe grid position
# 
#
#         
#####################################################

import cv2
import threading
import numpy as np
import os
import sys
import cv_bridge
from copy import deepcopy

import matplotlib.pylab as plt
import matplotlib.cm as cm

import math

import baxter_interface
from sensor_msgs.msg import Image

class GridDetector(object):
    
    def __init__(self, bw_img, tmpl_bw_img):
        
        self.bw_img = deepcopy(bw_img)
        self.image_width, self.image_height, self.image_channel = bw_img.shape
        
        self.tmpl_bw_img = deepcopy(tmpl_bw_img)
        self.tmpl_width, self.tmpl_height, self.tmpl_channel = tmpl_bw_img.shape
        
        
        
    
    
    def contour_matching(self):
    
    
    def df_matching(self):
    
    


