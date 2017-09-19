#!/usr/bin/python

########################################################
# @package baxterbase
#
# This file is to provide a base class to use 
# Baxter robot. Callback function for Image capture,
# IR sensor, robot endpoints are provided
#
# @copyright  2015-2017 UCRoboticsLab, All rights reserved.         
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
    '''!
    @brief The base class for baxter control. 
    '''
    
    def __init__(self):
        '''
        [YY]Initialize base class and create ROS publisher to topic 'robot_arms_cmd' 
        '''
        self.ArmCmdPub = rospy.Publisher('robot_arms_cmd', String, queue_size=10)
        self.OcvBridge = CvBridge()
        self.init_camera()
        self.init_endpoints()
        left = baxter_interface.Gripper('left', CHECK_VERSION)
        right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.baxter_grippers = {'left':left, 'right':right}
                

    def gripper_control(self, side, action):
        '''
        [YY]Perform open, close and calibrate with gripper. 
        @param side: select the gripper to take action
        @param action: the action to take
        '''
        gripper = self.baxter_grippers[side]
        if action == 'calibrate':
            gripper.calibrate()
        elif action == 'open':
            gripper.open()
        elif action == 'close':
            gripper.close()
            
    def init_camera(self):  
        '''
        [YY]Open, Configure and initialize hand cameras
        Resolution: 960 x 600
        Projection estimation of Calibration. 2.5mm per pixel at 1 meter distance
        '''
        
        self.ImageThreadLockLeft = threading.Lock()
        self.ImageThreadLockRight = threading.Lock()
        
        left_camera_sub = rospy.Subscriber('/cameras/left_hand_camera/image', \
                                       Image, self._left_camera_callback)
        
        right_camera_sub = rospy.Subscriber('/cameras/right_hand_camera/image', \
                                       Image, self._right_camera_callback)      
        # [YY] No reference to self._camera found in the project. May be removed. 
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

        # [YY] Initialize left and right current image object. Right only currently.        
        self.left_cur_img = None #np.zeros((self.height, self.width, 3), np.uint8)
        self.right_cur_img = np.zeros((self.height, self.width, 3), np.uint8)
        
                              
        
    def _left_camera_callback(self, image):
        '''
        [YY]Listener callback function to '/cameras/left_hand_camera/image'
        @param image: The ROS image message 
        @type image: sensor_msgs::Image
        '''
        with self.ImageThreadLockLeft:
            try:
                self.left_cur_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - LEFT IMAGE WENT WRONG!!'
        
    def _right_camera_callback(self, image):
        '''
        [YY]Listener callback function to '/cameras/right_hand_camera/image'
        @param image: The ROS image message 
        @type image: sensor_msgs::Image
        '''
        with self.ImageThreadLockRight:
            try:
                self.right_cur_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - RIGHT IMAGE WENT WRONG!!'

    
    def init_endpoints(self):
        '''
        [YY] Initialize end point listener for both arms. The state from both arms are 
        synchronized to update current endpoint pose.  
        '''
        self.PoseThreadLock = threading.Lock()
        self.current_poses = {'left':'', 'right':''}
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        ts = message_filters.ApproximateTimeSynchronizer([left_arm_msg, right_arm_msg], 10, 0.05)
        ts.registerCallback(self._pose_callback)


    def _pose_callback(self, left_msg, right_msg):
        '''! [YY] Update current pose for both arms.
        @param left_msg: msg for left arm
        @param right_msg: msg for right arm
        '''
    
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
        
        with self.PoseThreadLock:
            self.current_poses = {'left':cp1, 'right':cp2}
        
        
        #print self.current_poses
        #print "\nleft: \n", cp1, "\n\nnew left: \n", cp2
            
        return
        
    # def make_pose_stamp(self, pose_list, header): ----------------------------
#  -----------------------------------------------------------------------------
        # ps = PoseStamped() ---------------------------------------------------
        # ps.header = header ---------------------------------------------------
        # ps.pose.position = Point(x = pose_list[0], \ -------------------------
                                 # y = pose_list[1], \ -------------------------
                                 # z = pose_list[2], ) -------------------------
#  -----------------------------------------------------------------------------
        # ps.pose.orientation = Quaternion(x = pose_list[3], \ -----------------
                                         # y = pose_list[4], \ -----------------
                                         # z = pose_list[5], \ -----------------
                                         # w = pose_list[6], ) -----------------
#  -----------------------------------------------------------------------------
        # return ps ------------------------------------------------------------
    
    def get_img(self, side):
        '''
        [YY] Return the camera image in a thread safe way. 
        @param side: The side of arms from which to obtain the camera image 
        @type side: String
        '''
        
        c_img = None
        if side == 'left':
        
            with self.ImageThreadLockLeft:
                c_img = deepcopy(self.left_cur_img)
        elif side == 'right':
            with self.ImageThreadLockRight:
                c_img = deepcopy(self.right_cur_img)
        
        return c_img
    
    def get_pose(self, side):
        '''
        [YY] The thread safe way to get a snapshot of current pose
        TODO: There is a problem in this function. The function returns a 
        PoseStamped class and it's mutable. So it technically returns a reference 
        to the current pose data structure, instead of a copy. This defect renders
        the whole function pointless. A deepcopy should be made instead.    
        @param side: The side of arms from which to obtain the camera image 
        @type side: String
        '''
        
        cur_pose = None
        with self.PoseThreadLock:
            cur_pose = self.current_poses[side]
        
        return cur_pose 
    
    def make_pose_stamp(self, pose_list, header):
        '''
        Compose a time stamped pose message structure. 
        @param pose_list: pose coordinates 
        @type pose_list: list [x, y, z, quaternion_x, quaternion_y, quaternion_z, quaternion_w]
        @param header: message header
        @type header: 
        '''
        
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
    
    def init_msgs(self):
        '''
        [YY] This is a duplicate of init_endpoints
        TODO: remove this function.
        '''
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        ts = message_filters.ApproximateTimeSynchronizer([left_arm_msg, right_arm_msg], 10, 0.05)
        ts.registerCallback(self.pose_callback)
    
    def pose_callback(self, left_msg, right_msg):
        '''
        This function is duplicate of _pose_callback
        TODO: remove this function
        @param left_msg:
        @type left_msg:
        @param right_msg:
        @type right_msg:
        '''
    
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
    
    def move_arm(self, side, target_pose):
        '''
        [YY]Move arm to the target pose and then return 
        TODO: Check the outcome at the end of the function. The current 
        mechanism did not keep sending/publishing request to 
        @param side: which side of arm
        @type side: string
        @param target_pose: the target pose list
        @type target_pose: list
        '''
        # if e stop is on
        # [YY]do nothing if post is not a list of 7
        if len(target_pose)!=7:
            print "Move arm target pose list number not correct..."
            print "Error target pose: ", target_pose
            return
        
        x = target_pose[0]
        y = target_pose[1]
        z = target_pose[2]
        ox = target_pose[3]
        oy = target_pose[4]
        oz = target_pose[5]
        ow = target_pose[6]
        
        #[YY] make command string
        msg_string = side + ':move_to:' + \
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
        self.ArmCmdPub.publish(msg_string)
        #print self.current_poses
        #TODO: use get_pose to retrieve current pose in thread safe way. 
        cur_pose = self.current_poses[side]
        #TODO: vectorize the following block 
        dist_x = math.fabs(cur_pose.pose.position.x-x)
        dist_y = math.fabs(cur_pose.pose.position.y-y)
        dist_z = math.fabs(cur_pose.pose.position.z-z)
        dist = math.sqrt(dist_x*dist_x+dist_y*dist_y+dist_z*dist_z)
        
        #[YY] return after goal pose reached or 5 seconds
        counter = 0
        while dist>0.005: #dist_x>0.005 or dist_y>0.005 or dist_z>0.005:
            
            cur_pose = self.current_poses[side]
            
            dist_x = math.fabs(cur_pose.pose.position.x-x)
            dist_y = math.fabs(cur_pose.pose.position.y-y)
            dist_z = math.fabs(cur_pose.pose.position.z-z)
            dist = math.sqrt(dist_x*dist_x+dist_y*dist_y+dist_z*dist_z)
            if counter > 100:
                return
            counter = counter +1
            rospy.sleep(0.05)
        #print "Postion Reached", dist_x, dist_y, dist_z
        #print target_pose
        
    def pick_item(self, side, pick_pose):
        
        x = pick_pose[0]
        y = pick_pose[1]
        z = pick_pose[2]
        ox = pick_pose[3]
        oy = pick_pose[4]
        oz = pick_pose[5]
        ow = pick_pose[6]
        
        self.gripper_control(side, 'open')
        self.move_arm(side, [x, y, z+0.15, ox, oy, oz, ow])
        rospy.sleep(2)
        self.move_arm(side, [x, y, z+0.1, ox, oy, oz, ow])
        rospy.sleep(1)
        self.move_arm(side, [x, y, z, ox, oy, oz, ow])
        rospy.sleep(1)
        self.gripper_control(side, 'close')
        rospy.sleep(1)
        
        self.move_arm(side, [x, y, z+0.2, ox, oy, oz, ow])
        rospy.sleep(2)
        
        
        
        
    

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)

def main():
    
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node("baxter_test")
    side = sys.argv[1]
    t1 = BaxterBase()
    #t1.init_camera()
    
    t1.gripper_control('right', 'calibrate')
    
    # [YY] The main loop to update hand camera image 10 times per second 
    # and take snapshot if ESC is hit.  
    while not rospy.is_shutdown():
            
        img = t1.get_img(side)
        if img != None:
            cv2.imshow('current_image', img)
            key = cv2.waitKey(5)
            if key == 27:
                cv2.imwrite("cur_img.png", img)
        #print "Left Pose: \n", t1.get_pose('left')
        #print "Right Pose: \n", t1.get_pose('right')

        
        rospy.sleep(0.1)


if __name__ == '__main__':
    sys.exit(main())
