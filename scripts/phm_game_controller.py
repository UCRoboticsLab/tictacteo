#!/usr/bin/python

##########################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to use the Baxter robot to play Tig Tag Toe
# game (as a central controller)
#
#         
##########################################################

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


class TigTagToe(object):
    
    def __init__(self):
        
        self.GameArray = np.zeros((3,3,1), np.uint8)
        self.ArmCmdPub = rospy.Publisher('robot_arms_cmd', String, queue_size=10)
        self.VisionCmdPub = rospy.Publisher('robot_vision_cmd', String, queue_size=10)
        self.VisionReply = ''
        self.ArmReply = ''
        rospy.init_node("game_controller")
        rospy.Subscriber('arm_reply', String, self.arms_reply_callback)
        rospy.Subscriber('vision_reply', String, self.vision_reply_callback)
        self.GridCenter = []
        self.LeftArmInitPose = [0.5, 0.6, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.RightArmInitPose = [0.5, -0.6, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.current_poses = None
        
    def get_vision_reply(self):    
        
        cv_reply = self.VisionReply
        
        while cv_reply == '':
            cv_reply = self.VisionReply
            rospy.sleep(0.1)
            
        self.VisionReply = ''
        return cv_reply
    
    def interpret_vision_reply(self, msg_string):
        msg_segment = msg_string.split(':')
        if len(msg_segment)!= 2:
            rospy.logerr("Vision Reply Failure: msg segment not right")
            return '', []
        item_name = msg_segment[0]
        item_pose = [float(m) for m in msg_segment[2].split(',')]
        
        return item_name, item_pose
        
    def move_arm(self, side, target_pose):
        
        x = target_pose[0]
        y = target_pose[1]
        z = target_pose[2]
        ox = target_pose[3]
        oy = target_pose[4]
        oz = target_pose[5]
        ow = target_pose[6]
        
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
        print self.current_poses
        cur_pose = self.current_poses[side]
        dist_x = math.fabs(cur_pose.pose.position.x-x)
        dist_y = math.fabs(cur_pose.pose.position.y-y)
        dist_z = math.fabs(cur_pose.pose.position.z-z)
        
        while dist_x>0.002 and dist_y>0.02 and dist_z>0.02:
            cur_pose = self.current_poses[side]
            
            dist_x = math.fabs(cur_pose.pose.position.x-x)
            dist_y = math.fabs(cur_pose.pose.position.y-y)
            dist_z = math.fabs(cur_pose.pose.position.z-z)
            rospy.sleep(0.1)
        print "Postion Reached"
    
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
    
    def init_msgs(self):
        
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        ts = message_filters.ApproximateTimeSynchronizer([left_arm_msg, right_arm_msg], 10, 0.05)
        ts.registerCallback(self.pose_callback)
    
    def pose_callback(self, left_msg, right_msg):
    
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
        
    def init_game(self):
        
        
        rospy.sleep(1)
        print "Game Init Starts"
        self.move_arm('left', self.LeftArmInitPose)
        rospy.sleep(2)
        self.move_arm('right', self.RightArmInitPose)
        rospy.sleep(2)
        self.move_arm('left', [0.6, -0.2, 0.1, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(2)
        
        msg_string = 'left:detect:grid'
        self.VisionCmdPub.publish(msg_string)
        
        cv_reply = self.get_vision_reply()
        print "Grid Tracking result: ", cv_reply
        item_name, item_pose = self.interpret_vision_reply(cv_reply)
        
        if item_name != '':
            self.GridCenter = item_pose
            
        msg_string = 'detect:x'
        self.VisionCmdPub.publish(msg_string)
        
        
        
    
    def run(self):
        
        self.init_game()
        print "Game Initilized"
        while not rospy.is_shutdown():
            
            rospy.sleep(0.1)
        
    
    def vision_reply_callback(self, msg):
        self.VisionReply = msg.data
        
    def arms_reply_callback(self, msg):
        self.ArmReply = msg.data
    
    
def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)
    
    
def main():

    signal.signal(signal.SIGINT, signal_handler)
    ttt_game = TigTagToe()
    ttt_game.init_msgs()
    
    ttt_game.run()
    

if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print "\nCtrl-C Pressed\n"
        flag = True
        sys.exit()    
    
    
    
    