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

class GridDetector(object):
    
    def __init__(self, tmpl_bw_img):
        
        self.current_poses = None
        
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
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0.045                      # camera gripper offset
        self.cam_y_offset = -0.01
        
        
        self.cur_img = None                             
        self.ImageThreadLock = threading.Lock()
        
        
        
    def init_msgs(self):
        
        left_ir_msg = message_filters.Subscriber('/robot/range/left_hand_range/state',Range)
        right_ir_msg = message_filters.Subscriber('/robot/range/right_hand_range/state',Range)
        ts = message_filters.ApproximateTimeSynchronizer([left_ir_msg, right_ir_msg], 10, 0.05)
        ts.registerCallback(self._ir_sensor_callback)
        
        self.current_ir_ranges = {'left':65.0, 'right':65.0}
        
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        ts = message_filters.ApproximateTimeSynchronizer([left_arm_msg, right_arm_msg], 10, 0.05)
        ts.registerCallback(self._pose_callback)
    
    def get_ir_range(self, side):
        range = self.current_ir_ranges[side]
        return range
    
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
        
        
        
        #print "\nleft: \n", cp1, "\n\nnew left: \n", cp11
            
        return
    
    def _ir_sensor_callback(self, left_msg, right_msg): #def _ir_sensor_callback(self, msg, side):
        
        #print "\nLeft IR: \n", left_msg.range
        self.current_ir_ranges={'left':left_msg.range, 'right':right_msg.range}
        return
    
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
        matching_result = []
        for cnt in contours:

            contour_area = cv2.contourArea(cnt)
            #if contour_area<100:
            #    matching_result.append(10.0) # a large number to fill the list
            #    continue
            empty_img = np.zeros((self.height,self.width,1), np.uint8)
            contour_img = cv2.merge((bw_img1, empty_img, empty_img)) #np.zeros((self.height,self.width,3), np.uint8)
            cv2.drawContours(contour_img, contours, counter, (255,255,255), 2)
            ret = cv2.matchShapes(cnt,self.tmpl_contours[0], cv2.cv.CV_CONTOURS_MATCH_I1, 0.0)
            #print ret, "\nContour Area: \n", contour_area
            if ret == 0.0:
                ret = 10.0
            matching_result.append(ret)
            #plot_img = np.zeros((self.height,self.width,3), np.uint8)
            
            counter = counter + 1
            #cv2.imshow('current_image', contour_img)
            #cv2.waitKey(0)
            #rospy.sleep(0.1)
        
        min_index = matching_result.index(min(matching_result))
        rect = cv2.minAreaRect(contours[min_index])
        angle = rect[2]
        cx = math.floor(rect[0][0])
        cy = math.floor(rect[0][1])
        print "\nMatching Result: (", matching_result[min_index], ")\nangle: ", angle
        empty_img = np.zeros((self.height,self.width,1), np.uint8)
        contour_img = cv2.merge((bw_img1, empty_img, empty_img))
        plot_img = deepcopy(img)
        cv2.drawContours(plot_img, contours, min_index, (255,0,0), 2)
        cv2.imshow('current_image', plot_img)
        cv2.waitKey(20)
        return cx, cy, angle
    
    def pixel_to_baxter(self, px, dist):
        
        x1 = self.current_poses['left'].pose.position.x 
        y1 = self.current_poses['left'].pose.position.y
        print "\nCurrent Pose x, y:", x1, y1, "\n"
        print "Current Cx, Cy: ", px[0], px[1], "\n"
        print "Current Distance: ", dist, "\n"
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist) \
           + self.cam_x_offset
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist) \
           + self.cam_y_offset
        
        return x, y
    
    def put_to_grid(self, x, y):
        pass
    
    
    def dt_matching(self, img):
        
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
        matching_result = []
        for cnt in contours:

            contour_area = cv2.contourArea(cnt)
            #if contour_area<100:
            #    matching_result.append(10.0) # a large number to fill the list
            #    continue
            empty_img = np.zeros((self.height,self.width,1), np.uint8)
            contour_img = cv2.merge((bw_img1, empty_img, empty_img)) #np.zeros((self.height,self.width,3), np.uint8)
            cv2.drawContours(contour_img, contours, counter, (255,255,255), 2)
            ret = cv2.matchShapes(cnt,self.tmpl_contours[0], cv2.cv.CV_CONTOURS_MATCH_I1, 0.0)
            #print ret, "\nContour Area: \n", contour_area
            if ret == 0.0:
                ret = 10.0
            matching_result.append(ret)
            #plot_img = np.zeros((self.height,self.width,3), np.uint8)
            
            counter = counter + 1
            #cv2.imshow('current_image', contour_img)
            #cv2.waitKey(0)
            #rospy.sleep(0.1)
    
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
        
    def move_to(self, x, y, z, ox, oy, oz, ow, arm):
        msg_string = arm + ':move_to:' + \
                      str(x) + \
                      ',' + \
                      str(y) + \
                      ',' + \
                      str(z) + \
                      ',' + \
                      str(ox) + \
                      ',' + \
                      str(oy) + \
                      ',' + \
                      str(oz) + \
                      ',' + \
                      str(ow)
        self.rb_cmd_pub.publish(msg_string)
        
        return
    
    def move_arm(self, x, y, z, arm):
        
        msg_string = arm + ':move:' + \
                      str(x) + \
                      ',' + \
                      str(y) + \
                      ',' + \
                      str(z) + \
                      ',' + \
                      str(ox) + \
                      ',' + \
                      str(oy) + \
                      ',' + \
                      str(oz) + \
                      ',' + \
                      str(ow)
        self.rb_cmd_pub.publish(msg_string)
        
        return
    def init_arm_angle(self, arm):
        msg_string = arm + ':init_angle:0.0,0.0,0.0,0.0,1.0,0.0,0.0'
            
        self.rb_cmd_pub.publish(msg_string)
        
        return
        
    def cv_show_img(self, img, key_pressed):
        
        cv2.imshow('current_image', img)
        key = cv2.waitKey(10)
        while key != key_pressed:
            key = cv2.waitKey(10)
            
        
flag = False
def main():
    
    cv2.namedWindow('current_image')
    signal.signal(signal.SIGINT, signal_handler)
    template_img = cv2.imread(sys.argv[1])
    gd = GridDetector(template_img) 
    
    rospy.init_node("phm_find_grid")
    gd.init_msgs()
    gd.init_arm_angle('left')
    img = gd.get_image()
    angle = 0 
    cx = 0
    cy = 0
##    if img != None:
##        cx, cy, angle = gd.contour_matching(img)
##    x, y = gd.pixel_to_baxter([cx, cy], gd.get_ir_range('left'))
##
##    gd.move_to(-round(x, 2), -round(y, 2), 0.0, 0.0, 0.0, 0.0, 0.0, left)
    
    while not rospy.is_shutdown() and not flag:
        
        img = gd.get_image()
        if img != None:
            cx, cy, angle = gd.contour_matching(img)
            #x, y = gd.pixel_to_baxter([cx, cy], gd.get_ir_range('left'))

            #gd.move_to(-round(x, 2), -round(y, 2), 0.0, 0.0, 0.0, 0.0, 0.0, left)

        
        
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
        

    
    


