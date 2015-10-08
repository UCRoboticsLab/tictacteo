#!/usr/bin/python

########################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to use the Baxter robot left arm camera
# to find tig tag toe grid position
# 
#
#         
########################################################

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
from sensor_msgs.msg import Image
import message_filters
from std_msgs.msg import String
import rospy
import signal

class GridDetector(object):
    
    def __init__(self, tmpl_bw_img):
        
        self.rb_cmd_pub = rospy.Publisher('robot_arms_cmd', String, queue_size=10)
        self.tmpl_width, self.tmpl_height, self.tmpl_channel = tmpl_bw_img.shape
        self.OcvBridge = CvBridge()
        if self.tmpl_channel==3:
            self.tmpl_bw_img, t1, t2 = cv2.split(tmpl_bw_img)
        elif self.tmpl_channel==1:
            self.tmpl_bw_img = deepcopy(tmpl_bw_img)
        else:
            self.tmpl_bw_img = None
        
        
        self.tmpl_contours, tmpl_hierarchy = cv2.findContours \
                                                (self.tmpl_bw_img, \
                                                 cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        left_camera_sub = rospy.Subscriber('/cameras/left_hand_camera/image', \
                                       Image, self._camera_callback)
                                    
        self.height = 600
        self.width = 960
        self._camera = baxter_interface.CameraController('left_hand_camera')
        self._camera.open()
        self._camera.resolution = [self.width, self.height]
        self._camera.gain = 10
        
        self.cur_img = None                             
        self.ImageThreadLock = threading.Lock()
        
    def _camera_callback(self, image):
        with self.ImageThreadLock:
            try:
                self.cur_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - IMAGE WENT WRONG!!'
        
    def get_image(self):
        c_img = None
        with self.ImageThreadLock:
            c_img = deepcopy(self.cur_img)
        
        return c_img
        
        
    def contour_matching(self, img):
    
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kernel_smooth = np.ones((5,5),np.float32)/25
        gray_img = cv2.filter2D(gray_img, -1, kernel_smooth)
        
        bw_img = cv2.adaptiveThreshold(gray_img,255,\
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                                       cv2.THRESH_BINARY,103,1)
                                       
                                       
        bw_img1 = cv2.bitwise_not(bw_img)
        
        bw_img2 = deepcopy(bw_img1)
        contours, hierarchy = cv2.findContours(bw_img2, cv2.RETR_TREE, \
                                               cv2.CHAIN_APPROX_SIMPLE)
        counter = 0
        
        print "\n...Start with a new image...\n"
        for cnt in contours:

            contour_area = cv2.contourArea(cnt)
            if contour_area<100:
                continue
            empty_img = np.zeros((self.height,self.width,1), np.uint8)
            contour_img = cv2.merge((bw_img1, empty_img, empty_img)) #np.zeros((self.height,self.width,3), np.uint8)
            cv2.drawContours(contour_img, contours, counter, (255,255,255), 1)
            ret = cv2.matchShapes(cnt,self.tmpl_contours[0], cv2.cv.CV_CONTOURS_MATCH_I1, 0.0)
            print ret, "\nContour Area: \n", contour_area

            #plot_img = np.zeros((self.height,self.width,3), np.uint8)
            
            counter = counter + 1
            cv2.imshow('current_image', contour_img)
            cv2.waitKey(0)
            #rospy.sleep(0.1)
        
       
    
    
    
    
    def dt_matching(self, img):
        pass
        

flag = False
def main():
    
    cv2.namedWindow('current_image')
    signal.signal(signal.SIGINT, signal_handler)
    template_img = cv2.imread(sys.argv[1])
    gd = GridDetector(template_img) 
    
    rospy.init_node("phm_find_grid")
    
    while not rospy.is_shutdown() and not flag:

        img = gd.get_image()
        if img != None:
            gd.contour_matching(img)
        
        rospy.sleep(0.1)

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)


if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print "\nCtrl-C Pressed\n"
        flag = True
        sys.exit()
        

    
    


