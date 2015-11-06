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
    
    def __init__(self, arm):
        
        self.arm = arm
        self.current_task_dropping = False
        self.current_poses = None
        self.current_vision_cmd = ''
        self.rb_cmd_pub = rospy.Publisher('vision_reply', String, queue_size=10)
        self.template_imgs = {}
        #self.tmpl_width, self.tmpl_height, self.tmpl_channel = tmpl_bw_img.shape
        self.OcvBridge = CvBridge()
        #if self.tmpl_channel==3:
        #    self.tmpl_bw_img, t1, t2 = cv2.split(tmpl_bw_img)
        #elif self.tmpl_channel==1:
        #    self.tmpl_bw_img = deepcopy(tmpl_bw_img)
        #else:
        #    self.tmpl_bw_img = None
        
        rospy.Subscriber('robot_vision_cmd', String, self.vision_cmd_callback)
        
        
        
        
        
        left_camera_sub = rospy.Subscriber('/cameras/'+ self.arm + '_hand_camera/image', \
                                       Image, self._camera_callback)
                                    
        self.height = 600
        self.width = 960
        self._camera = baxter_interface.CameraController(self.arm + '_hand_camera')
        
        
        self._camera.open()
        self._camera.resolution = [self.width, self.height]
        self._camera.gain = 10
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0.045                      # camera gripper offset
        self.cam_y_offset = -0.01
        
        
        self.cur_img = None                             
        self.ImageThreadLock = threading.Lock()
        
        self.image_names = {'grid':'template_grid_white_center.png', \
                            'o':'template_grid_white_center.png', \
                            'x':'template_x01.png'}
                            
        self.image_folder = './src/phm/images/'
        
    def vision_cmd_callback(self, msg):
        self.current_vision_cmd = msg.data
        
    def add_template_images(self):
        types = self.image_names.keys()
        for type in types:
            filename = self.image_folder + self.image_names[type]
            print "add template image" + filename
            tmpl_img = cv2.imread(filename)
            tmpl_width, tmpl_height, tmpl_channel = tmpl_img.shape
            
            tmpl_bw_img = None
            if tmpl_channel==3:
                tmpl_bw_img, t1, t2 = cv2.split(tmpl_img)
            elif tmpl_channel==1:
                tmpl_bw_img = deepcopy(tmpl_img)
            else:
                tmpl_bw_img = None
                return
            
            self.template_imgs.update({type:tmpl_bw_img})
        
        return
        
    def interpret_vision_cmd(self, msg_string):
        
        msg_segments = msg_string.split(':')        
        if len(msg_segments) != 3:
            return '', '', ''
        
        side = msg_segments[0]
        action = msg_segments[1]
        target = msg_segments[2]
        
        return side, action, target
        
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
        
    '''    
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

            #contour_area = cv2.contourArea(cnt)
            mom = cv2.moments(cnt)
            contour_area = mom['m00']
            #print "\nContourArea: ", contour_area, "Moments: ", contour_area1
            #if contour_area<100:
            #    matching_result.append(10.0) # a large number to fill the list
            #    continue
            empty_img = np.zeros((self.height,self.width,1), np.uint8)
            contour_img = cv2.merge((bw_img1, empty_img, empty_img)) #np.zeros((self.height,self.width,3), np.uint8)
            cv2.drawContours(contour_img, contours, counter, (255,255,255), 2)
            ret = cv2.matchShapes(cnt,self.tmpl_contours[0], cv2.cv.CV_CONTOURS_MATCH_I1, 0.0)
            #print ret, "\nContour Area: \n", contour_area
            if ret == 0.0 or contour_area<300:
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
    '''
    
    def contour_match(self, img, tmpl_bw_img, mask_img, rect):
        
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kernel_smooth = np.ones((5,5),np.float32)/25
        gray_img = cv2.filter2D(gray_img, -1, kernel_smooth)
        
        bw_img = cv2.adaptiveThreshold(gray_img,255,\
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                                       cv2.THRESH_BINARY,103,1)
                                    
        bw_img1 = cv2.bitwise_not(bw_img, mask = mask_img)
        #cv2.imshow('current_image', bw_img1)
        #cv2.waitKey(10)
        bw_img2 = deepcopy(bw_img1)
        contours, hierarchy = cv2.findContours(bw_img2, cv2.RETR_TREE, \
                                               cv2.CHAIN_APPROX_SIMPLE)
        
        tmpl_contours, tmpl_hierarchy = cv2.findContours \
                                                (tmpl_bw_img, \
                                                 cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                                    
                                            
        counter = 0
        matching_result = []
        for cnt in contours:
            
            mom = cv2.moments(cnt)
            contour_area = mom['m00']
            
            ret = cv2.matchShapes(cnt,tmpl_contours[0], cv2.cv.CV_CONTOURS_MATCH_I1, 0.0)
            #print ret, "\nContour Area: \n", contour_area
            if ret == 0.0 or contour_area<300:
                ret = 10.0
            matching_result.append([ret,cnt])
            
            counter = counter + 1
        sorted_list = sorted(matching_result,key=lambda x: x[0])        
        new_list = sorted_list[0:5]
        if new_list[0][0]>0.1:
            print "Can't find item"
            cv2.imshow('current_image', img)
            cv2.waitKey(10)
            return None
        dist_list = []
        new_rect = None
        plot_img = deepcopy(img)
        if rect == None:
            print "None Image"
            new_rect = cv2.minAreaRect(new_list[0][1])
            cv2.drawContours(plot_img, [new_list[0][1]], 0, (255,0,0), 2)
        
        else:
            print "Real Images"
            cx0 = rect[0][0]
            cy0 = rect[0][1]
            
            for item in new_list:
                
                cnt = item[1]
                new_rect1 = cv2.minAreaRect(cnt)
                cx1 = new_rect1[0][0]
                cy1 = new_rect1[0][1]
                dist = math.sqrt((cx0-cx1)*(cx0-cx1) + (cy0-cy1)*(cy0-cy1))
                dist_list.append([dist, new_rect1, cnt])
        
            
            #min_index = matching_result.index(min(matching_result))
            #print "size of dist list", len(dist_list)
            sorted_dist_list = sorted(dist_list,key=lambda x: x[0])
            new_rect = sorted_dist_list[0][1]
            cv2.drawContours(plot_img, [sorted_dist_list[0][2]], 0, (255,0,0), 2)
        
        cv2.imshow('current_image', plot_img)
        cv2.waitKey(10)
        #sorted_dist_list 
        #rect = cv2.minAreaRect(contours[min_index])
        #angle = rect[2]
        #cx = math.floor(rect[0][0])
        #cy = math.floor(rect[0][1])
        
        return new_rect
        
    #def get_new_rect(self, rect):
        
        
    def integer_box(self, box, width, height):
        
        
        x0 = math.floor(box[0][0])
        y0 = math.floor(box[0][1])
        
        x1 = math.floor(box[1][0])
        y1 = math.floor(box[1][1])
        
        x2 = math.floor(box[2][0])
        y2 = math.floor(box[2][1])
        
        x3 = math.floor(box[3][0])
        y3 = math.floor(box[3][1])
            
        int_box = np.array([ [x0, y0], [x1, y1], [x2, y2], [x3, y3] ], 'int32')
        
        for point in int_box:
            if point[0]>width:
                point[0] = width
                
            if point[1]>height:
                point[1] = height
                
            if point[0]<0:
                point[0] = 0
                
            if point[1]<0:
                point[1] = 0 
        
        
        return int_box
        
    def track_contour(self, side, object_type):
        img = self.get_image()
        if img == None:
            rospy.sleep(0.05)
            return
        
        height, width, channel = img.shape
        mask_img = np.ones((height,width,1), np.uint8)
        
        tmpl_bw_img = self.template_imgs[object_type]
        rect = self.contour_match(img, tmpl_bw_img, mask_img, None)
        #print rect
        while not self.current_task_dropping:
            
            img = self.get_image()
            if img == None:
                rospy.sleep(0.05)
                continue
            
            #new_h = rect[1][0]+rect[1][0]/2.0
            #new_w = rect[1][1]+rect[1][1]/2.0
            #center = (rect[0][0], rect[0][1])
            #size = (new_h, new_w)
            #new_rect = (center, size, rect[2])
            #box = cv2.cv.BoxPoints(new_rect)
            #int_box = self.integer_box(box, width, height)
            #print type(box), box
            #temp_img = np.zeros((height,width,3), np.uint8)
            #cv2.rectangle(temp_img, tuple(int_box[0]), tuple(int_box[2]), (255, 255, 255), -1, 8, 0)
            #mask_img, g, r = cv2.split(temp_img)
            #new_img = cv2.bitwise_and(img,img,mask = mask_img)
            
            rect1 = self.contour_match(img, tmpl_bw_img, mask_img, rect)
            rect = rect1
            if rect != None: 
                dx, dy = self.pixel_to_baxter([rect[0][0], rect[0][1]], self.get_ir_range(side))
                msg_string = side + \
                              object_type + \
                              ':' + str(round(dx, 4)) + ',' + str(round(dy, 4)) + \
                              ',' + str(round(self.get_ir_range(side), 4))
                self.rb_cmd_pub.publish(msg_string)
            #cv2.imshow('current_image', new_img)
            #cv2.waitKey(10)
            rospy.sleep(0.05)
            
            

    def pixel_to_baxter(self, px, dist):
        
        x1 = self.current_poses[self.arm].pose.position.x 
        y1 = self.current_poses[self.arm].pose.position.y
        print "\nCurrent Pose x, y:", x1, y1, "\n"
        print "Current Cx, Cy: ", px[0], px[1], "\n"
        print "Current Distance: ", dist, "\n"
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist) \
           + self.cam_x_offset 
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist) \
           + self.cam_y_offset 
        print "Current dx, dy: ", x, y 
        print "Target x, y: ", -x+x1, -y+y1
        #print "Target x, y: ", x+x1, y+y1
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

            #contour_area = cv2.contourArea(cnt)
            mom = cv2.moments(cnt)
            contour_area = mom
            #if contour_area<100:
            #    matching_result.append(10.0) # a large number to fill the list
            #    continue
            empty_img = np.zeros((self.height,self.width,1), np.uint8)
            contour_img = cv2.merge((bw_img1, empty_img, empty_img)) #np.zeros((self.height,self.width,3), np.uint8)
            cv2.drawContours(contour_img, contours, counter, (255,255,255), 2)
            ret = cv2.matchShapes(cnt,self.tmpl_contours[0], cv2.cv.CV_CONTOURS_MATCH_I3, 0.0)
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
                      str(0.0) + \
                      ',' + \
                      str(0.0) + \
                      ',' + \
                      str(0.0) + \
                      ',' + \
                      str(0.0)
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
            

    def run(self):
        
        while not rospy.is_shutdown():
            
            c_cmd = self.current_vision_cmd
            if c_cmd != '':
                self.current_vision_cmd = ''
                rospy.sleep(0.1)
                continue
            
            side, action, target = self.interpret_vision_cmd(c_cmd)
            
            if action == 'grid':
                
                pass
                
            elif action == 'x':
                pass
            
            elif action == 'o':
                pass
            
            
            
            
            rospy.sleep(0.1)
            
    
        
flag = False
def main():
    
    cv2.namedWindow('current_image')
    signal.signal(signal.SIGINT, signal_handler)
    #template_img = cv2.imread(sys.argv[1])
    arm = 'left'
    gd = GridDetector(arm)
    
    rospy.init_node("phm_find_grid")
    gd.init_msgs()
    
    gd.add_template_images()
    
    #gd.init_arm_angle('left')
    #img = gd.get_image()
    #angle = 0 
    #cx = 0
    #cy = 0
    #if img != None:
    #    cx, cy, angle = gd.contour_matching(img)
    #x, y = gd.pixel_to_baxter([cx, cy], gd.get_ir_range('left'))
    #gd.move_arm(0.0, 0.3, 0.0, 'left')
    #rospy.sleep(2)
    #gd.move_arm(round(x, 2), round(y, 2), 0.0, 'right')
    #gd.move_to(-round(x, 2), -round(y, 2), 0.0, 0.0, 0.0, 0.0, 0.0, 'left')
    #rospy.sleep(5)
    
    while not rospy.is_shutdown() and not flag:
        
        #img = gd.get_image()
        #if img != None:
        #    cx, cy, angle = gd.contour_matching(img)
            #x, y = gd.pixel_to_baxter([cx, cy], gd.get_ir_range('left'))

            #gd.move_to(-round(x, 2), -round(y, 2), 0.0, 0.0, 0.0, 0.0, 0.0, left)

        gd.track_contour(arm, 'grid')
        
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
        

    
    


