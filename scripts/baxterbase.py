#!/usr/bin/python

########################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to provide a base class to use 
# Baxter robot. Callback function for Image capture,
# IR sensor, robot endpoints are provided
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

class BaxterBase:
    
    def __init__(self):
        
        
        
        
        self.OcvBridge = CvBridge()
        self.init_camera()
        self.init_endpoints()
                

    
    def init_camera(self):
        
        self.ImageThreadLockLeft = threading.Lock()
        self.ImageThreadLockRight = threading.Lock()
        
        left_camera_sub = rospy.Subscriber('/cameras/left_hand_camera/image', \
                                       Image, self._left_camera_callback)
        
        right_camera_sub = rospy.Subscriber('/cameras/right_hand_camera/image', \
                                       Image, self._right_camera_callback)      
        
        self.left_camera = self._camera = baxter_interface.CameraController('left_hand_camera')
        
        
                                    
        self.right_camera = self._camera = baxter_interface.CameraController('right_hand_camera')
        
        self.height = 600
        self.width = 960        
        
        self.left_camera.open()
        self.left_camera.resolution = [self.width, self.height]
        self.left_camera.gain = 5
        
        self.right_camera.open()
        self.right_camera.resolution = [self.width, self.height]
        self.right_camera.gain = 5
        
        
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0.045                      # camera gripper offset
        self.cam_y_offset = -0.01
        
        
        
        self.left_cur_img = None #np.zeros((self.height, self.width, 3), np.uint8)
        self.right_cur_img = np.zeros((self.height, self.width, 3), np.uint8)
        
                              
        
    def _left_camera_callback(self, image):
        
        with self.ImageThreadLockLeft:
            try:
                self.left_cur_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - LEFT IMAGE WENT WRONG!!'
        
    def _right_camera_callback(self, image):
        
        with self.ImageThreadLockRight:
            try:
                self.right_cur_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - RIGHT IMAGE WENT WRONG!!'

    
    def init_endpoints(self):
        
        self.current_poses = None
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        ts = message_filters.ApproximateTimeSynchronizer([left_arm_msg, right_arm_msg], 10, 0.05)
        ts.registerCallback(self._pose_callback)


    def _pose_callback(self, left_msg, right_msg):
    
        pose1 = left_msg.pose
        pose2 = right_msg.pose
        
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        cp1 = self.make_pose_stamp([pose1.position.x, \
                                     pose1.position.y, \
                                     pose1.position.z, \
                                     pose1.orientation.x, \
                                     pose1.orientation.y, \
                                     pose1.orientation.z, \
                                     pose1.orientation.w], \
                                     header)
        
        cp2 = self.make_pose_stamp([pose2.position.x, \
                                     pose2.position.y, \
                                     pose2.position.z, \
                                     pose2.orientation.x, \
                                     pose2.orientation.y, \
                                     pose2.orientation.z, \
                                     pose2.orientation.w], \
                                     header)
        
        self.current_poses = {'left':cp1, 'right':cp2}
        
        
        #print self.current_poses
        #print "\nleft: \n", cp1, "\n\nnew left: \n", cp2
            
        return
        
    def make_pose_stamp(self, pose_list, header):
        
        ps = PoseStamped()
        ps.header = header
        ps.pose.position = Point(x = pose_list[0], \
                                 y = pose_list[1], \
                                 z = pose_list[2], )

        ps.pose.orientation = Quaternion(x = pose_list[3], \
                                         y = pose_list[4], \
                                         z = pose_list[5], \
                                         w = pose_list[6], )
        
        return ps 
    
    def get_img(self, side):
        
        c_img = None
        if side == 'left':
        
            with self.ImageThreadLockLeft:
                c_img = deepcopy(self.left_cur_img)
        elif side == 'right':
            with self.ImageThreadLockRight:
                c_img = deepcopy(self.right_cur_img)
        
        return c_img
    
    def get_pose(self, side):
        cur_pose = self.current_poses[side]

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)

def main():
    
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("baxter_test")
    
    t1 = BaxterBase()
    #t1.init_camera()
    
    
    
    while not rospy.is_shutdown():
            
        img = t1.get_img('right')
        if img != None:
            cv2.imshow('current_image', img)
            cv2.waitKey(5)
        rospy.sleep(0.1)


if __name__ == '__main__':
    sys.exit(main())
