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
import baxter_external_devices
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from baxter_core_msgs.msg import EndpointState
from baxter_core_msgs.msg import AssemblyState
#from baxter_interface import DigitalIO
from baxter_core_msgs.msg import DigitalIOState
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
from random import randint



class TigTagToe(object):
    
    def __init__(self):
        
        self.GameArray = np.zeros((3,3,1), np.uint8)
        self.ArmCmdPub = rospy.Publisher('robot_arms_cmd', String, queue_size=10)
        self.VisionCmdPub = rospy.Publisher('robot_vision_cmd', String, queue_size=10)
        self.GridStatusPub = rospy.Publisher('game_engine_cmd', String, queue_size=10)
        self.VisionReply = ''
        self.ArmReply = ''
        self.NextMoveReply = ''
        self.LeftSlots = ['x','x','x','x','x']
        self.RightSlots = ['o','o','o','o','o']
        self.GridStatus = ['b','b','b','b','b','b','b','b','b']
        self.FirstPlayMarker = ''
        self.LeftSlotsLocation = []
        self.RightSlotsLocation = []
        rospy.init_node("game_controller")
        rospy.Subscriber('arm_reply', String, self.arms_reply_callback)
        rospy.Subscriber('vision_reply', String, self.vision_reply_callback)
        rospy.Subscriber('next_move', String, self.next_move_callback)
        rospy.Subscriber('/robot/digital_io/right_shoulder_button/state', DigitalIOState, self.button_callback)
        rospy.Subscriber('/robot/state', AssemblyState, self.robot_state_callback)
        
        self.ButtonStatus = 0
        self.EstopStatus = 0
        self.GridCenter = []
        self.TableHeight = 0.0
        self.GripperLength = 0.125
        self.LeftArmInitPose = [0.4, 0.65, 0.15, 0.0, 1.0, 0.0, 0.0]
        self.RightArmInitPose = [0.4, -0.65, 0.15, 0.0, 1.0, 0.0, 0.0]
        self.current_poses = None
        self.GridLocations = []
        self.GridRoiLocations = []
        self.GridRoiPose = []
        self.TableZ = 100.0
        self.BlockHeight = 0.03
        self.GameState = 'NoInit'
        
        #rs = baxter_interface.RobotEnable(CHECK_VERSION)
        
        left = baxter_interface.Gripper('left', CHECK_VERSION)
        right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.baxter_grippers = {'left':left, 'right':right}
        
        left_arm = baxter_interface.Limb("left")
        right_arm = baxter_interface.Limb("right")
        self.arms = {'left':left_arm, 'right':right_arm}
        
        #pick
        self.LeftSlotsLocation = [ [0.3951+0.007, 0.2694, -0.080, 0.0147, 0.9999, -0.0085, -0.0051], \
                                    [0.5089, 0.2679, -0.080, -0.0001, 0.9996, -0.0093, -0.0243], \
                                    [0.6177+0.003, 0.2716-0.002, -0.080, 0.0166, 0.9995, -0.0033, -0.0266], \
                                    [0.5031+0.003, 0.3795+0.003, -0.080, 0.0153, 0.9998, -0.0091, -0.0435], \
                                    [0.6150+0.006, 0.3809, -0.080, 0.0033, 0.9998, 0.0030, -0.0086] ]
        #place                            
        self.LeftSlotsLocation1 = [ [0.3951+0.004, 0.2694, -0.080, 0.0147, 0.9999, -0.0085, -0.0051], \
                                    [0.5089-0.005, 0.2679+0.005, -0.080, -0.0001, 0.9996, -0.0093, -0.0243], \
                                    [0.6177-0.013, 0.2716+0.003, -0.080, 0.0166, 0.9995, -0.0033, -0.0266], \
                                    [0.5031-0.002, 0.3795, -0.080, 0.0153, 0.9998, -0.0091, -0.0435], \
                                    [0.6150+0.008, 0.3809+0.002, -0.080, 0.0033, 0.9998, 0.0030, -0.0086] ]
        #pick
        self.RightSlotsLocation = [ [0.4040+0.015, -0.2569-0.002, -0.076, -0.0206, 0.9997, 0.0018, 0.0164], \
                                    [0.5131+0.016, -0.2545-0.001, -0.076, -0.0057, 0.9998, 0.0029, 0.0198], \
                                    [0.6174+0.015, -0.2559-0.001, -0.076, -0.0179, 0.9996, -0.0142, -0.0104], \
                                    [0.5224+0.008, -0.3682+0.002, -0.076, -0.0186, 0.9997, 0.0058, -0.0127], \
                                    [0.6285+0.012, -0.3660+0.001, -0.076, 0.0031, 0.9998, 0.0031, 0.0191] ]
        #place                            
        self.RightSlotsLocation1 = [ [0.4040+0.005, -0.2569-0.002, -0.076, -0.0206, 0.9997, 0.0018, 0.0164], \
                                    [0.5131+0.01, -0.2545-0.003, -0.076, -0.0057, 0.9998, 0.0029, 0.0198], \
                                    [0.6174+0.015, -0.2559, -0.076, -0.0179, 0.9996, -0.0142, -0.0104], \
                                    [0.5224, -0.3682, -0.076, -0.0186, 0.9997, 0.0058, -0.0127], \
                                    [0.6285+0.005, -0.3660-0.008, -0.076, 0.0031, 0.9998, 0.0031, 0.0191] ]
        #pick
        self.GridForLeftArm = [ [0.3965, -0.1069+0.004, -0.076, 0.03617, 0.9991, -0.0098, -0.0198], \
                                [0.5040+0.01, -0.1058-0.002, -0.076, 0.0147, 0.9994, 0.0002, -0.0316], \
                                [0.6115+0.007, -0.0908-0.022, -0.076, 0.0146, 0.9999, 0.0064, -0.0047], \
                                [0.3917+0.004, 0.0117-0.01, -0.076, 0.0073, 0.9993, 0.0042, -0.0364], \
                                [0.4983+0.009, 0.0108-0.01, -0.076, -0.0066, 0.9999, 0.0015, -0.0127], \
                                [0.6093+0.013, 0.0097-0.011, -0.076, 0.0154, 0.9997, 0.0029, -0.0194], \
                                [0.3914+0.004, 0.1200-0.01, -0.076, 0.0126, 0.9994, -0.0002, -0.0333], \
                                [0.5029+0.006, 0.1203-0.011, -0.076, 0.0020, 0.9993, 0.0008, -0.0378], \
                                [0.6103+0.01, 0.1161-0.006, -0.076, 0.0195, 0.9992, -0.0010, -0.0352], \
        
                                ]
        #place                        
        self.GridForLeftArm1 = [ [0.3965, -0.1069-0.003, -0.076, 0.03617, 0.9991, -0.0098, -0.0198], \
                                [0.5040-0.005, -0.1058, -0.076, 0.0147, 0.9994, 0.0002, -0.0316], \
                                [0.6115+0.002, -0.0908-0.018, -0.076, 0.0146, 0.9999, 0.0064, -0.0047], \
                                [0.3917, 0.0117-0.01, -0.076, 0.0073, 0.9993, 0.0042, -0.0364], \
                                [0.4983+0.01, 0.0108-0.008, -0.076, -0.0066, 0.9999, 0.0015, -0.0127], \
                                [0.6093+0.01, 0.0097-0.008, -0.076, 0.0154, 0.9997, 0.0029, -0.0194], \
                                [0.3914+0.012, 0.1200-0.01, -0.076, 0.0126, 0.9994, -0.0002, -0.0333], \
                                [0.5029, 0.1203-0.008, -0.076, 0.0020, 0.9993, 0.0008, -0.0378], \
                                [0.6103+0.009, 0.1161-0.005, -0.076, 0.0195, 0.9992, -0.0010, -0.0352], \
        
                                ]
        #pick
        self.GridForRightArm =[ [0.4019+0.007, -0.0972, -0.076, -0.0221, 0.9997, 0.0069, -0.0032], \
                                [0.5125+0.002, -0.0957, -0.076, -0.0178, 0.9996, 0.0053, -0.0213], \
                                [0.6169+0.004, -0.0947-0.004, -0.076, -0.0172, 0.9996, -0.0009, 0.0209], \
                                [0.3964,  0.0073+0.005, -0.076, -0.0174, 0.9998, -0.0050, -0.0050], \
                                [0.5020+0.015, 0.0090+0.005, -0.076, -0.0068, 0.9998, -0.0050, 0.0195], \
                                [0.6169+0.006, 0.0147, -0.076, -0.0059, 0.9999, 0.0022, -0.0015], \
                                [0.3948+0.01, 0.1160+0.008, -0.076, -0.0070, 0.9998, -0.0034, -0.0180], \
                                [0.5042+0.002, 0.1188+0.006, -0.076, -0.0058, 0.9999, -0.0073, 0.0087], \
                                [0.6150+0.012, 0.1181+0.005, -0.076, -0.0235, 0.9995, -0.0124, 0.0146] ]
        #place                        
        self.GridForRightArm1 =[ [0.4019+0.007, -0.0972, -0.076, -0.0221, 0.9997, 0.0069, -0.0032], \
                                [0.5125+0.008, -0.0957, -0.076, -0.0178, 0.9996, 0.0053, -0.0213], \
                                [0.6169+0.012, -0.0947-0.001, -0.076, -0.0172, 0.9996, -0.0009, 0.0209], \
                                [0.3964+0.01,  0.0073+0.007, -0.076, -0.0174, 0.9998, -0.0050, -0.0050], \
                                [0.5020+0.015, 0.0090+0.006, -0.076, -0.0068, 0.9998, -0.0050, 0.0195], \
                                [0.6169+0.009, 0.0147, -0.076, -0.0059, 0.9999, 0.0022, -0.0015], \
                                [0.3948+0.002, 0.1160+0.01, -0.076, -0.0070, 0.9998, -0.0034, -0.0180], \
                                [0.5042+0.005, 0.1188+0.008, -0.076, -0.0058, 0.9999, -0.0073, 0.0087], \
                                [0.6150+0.012, 0.1181+0.005, -0.076, -0.0235, 0.9995, -0.0124, 0.0146] ]
                                
        self.RightSlots = ['o', 'o', 'o', 'o', 'o']
        self.LeftSlots = ['x', 'x', 'x', 'x', 'x']
        self.GridStatus = ['b', 'b', 'b', 'b', 'b', 'b', 'b', 'b', 'b']
        
    def robot_state_callback(self, msg):
        
        self.EstopStatus = msg
        if self.EstopStatus.estop_button == 1:
            self.GameState = 'Estop_on'
        
        return
        
    def button_callback(self, msg):
        
        #button_state = msg.data
        #print "Button State", msg.state
        self.ButtonStatus = msg.state
        return
    
    def wait_for_estop(self):
        
        if self.EstopStatus.enabled == True:
            return
        
        while self.EstopStatus.enabled == False and self.EstopStatus.estop_button == 1:
            rospy.sleep(0.1)
        
        #while self.ButtonStatus == 0:
        #    rospy.sleep(0.1)
        
        print "E Stop is reset"
        self.GameState = 'Estop_reset'
        
        return
        
        
        
        
    def get_button_status(self):
        
        button_status = self.ButtonStatus
        
        return button_status
    
    # wait for button on for second
    def wait_button_on(self, min_seconds, max_seconds):
        
        button_status = self.ButtonStatus
        counter = 0
        counter1 = 0
        max_count = math.floor(max_seconds/0.1)
        min_count = math.floor(min_seconds/0.1)
        while counter<max_count:
            
            if self.GameState == 'Estop_on':
                return -1
            button_status = self.ButtonStatus
            #print "Button Status: ", button_status
            if (button_status == 0):
                counter1 = counter1 + 1
            else:
                counter1 = 0
            rospy.sleep(0.1)
            counter = counter + 1
            if counter1>=min_count:
                return 1
            
        return 0 # return 0 : time out, return 1, button pressed correctly
            
        
    
    def get_vision_reply(self):
        
        cv_reply = self.VisionReply
        
        while cv_reply == '':
            
            if self.GameState== 'Estop_on':
                return ''
            cv_reply = self.VisionReply
            rospy.sleep(0.1)
            
        self.VisionReply = ''
        return cv_reply
    
    def interpret_vision_reply(self, msg_string):
        msg_segment = msg_string.split(':')
        if len(msg_segment)!= 3:
            rospy.logerr("Vision Reply Failure: msg segment not right")
            return '', []
        item_name = msg_segment[0]
        item_pose = [float(m) for m in msg_segment[2].split(',')]
        
        return item_name, item_pose
    
    def interpret_grid_checking_reply(self, msg_string):
        
        msg_segment = msg_string.split(':')
        print msg_segment
        if len(msg_segment)!= 3:
            rospy.logerr("Vision Reply Failure: msg segment not right")
            return '', []
        
        if msg_segment[1] != 'grid_status':
            rospy.logerr("Vision Reply Failure: msg type not right")
            return '', []
        
        grid_status = msg_segment[2].split()
        if len(grid_status) != 1:
            rospy.logerr("Vision Reply Failure: number of grids not right")
            return '', []
        
        return msg_segment[0], grid_status
    
    def interpret_grid_checking_reply1(self, msg_string):
        
        msg_segment = msg_string.split(':')
        print msg_segment
        if len(msg_segment)!= 4:
            rospy.logerr("Vision Reply Failure: msg segment not right")
            return '', [], []
        
        if msg_segment[1] != 'grid_status':
            rospy.logerr("Vision Reply Failure: msg type not right")
            return '', [], []
        
        grid_status = msg_segment[2].split()
        item_xy = msg_segment[3].split(',')
        xy_list = [float(item_xy[0]), float(item_xy[1])]
        #item_xy.remove('')
        if len(grid_status) != 1:
            rospy.logerr("Vision Reply Failure: number of grids not right")
            return '', [],[]
        
        return msg_segment[0], grid_status, xy_list
        
    def gripper_control(self, side, action):
        gripper = self.baxter_grippers[side]
        if action == 'calibrate':
            gripper.calibrate()
        elif action == 'open':
            gripper.open()
        elif action == 'close':
            gripper.close()
        
        
    def move_arm(self, side, target_pose):
        
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
        cur_pose = self.current_poses[side]
        dist_x = math.fabs(cur_pose.pose.position.x-x)
        dist_y = math.fabs(cur_pose.pose.position.y-y)
        dist_z = math.fabs(cur_pose.pose.position.z-z)
        
        counter = 0
        while dist_x>0.005 or dist_y>0.005 or dist_z>0.005:
            
            # if e stop is on
            if self.GameState == 'Estop_on':
                self.LeftSlots = ['x','x','x','x','x']
                self.RightSlots = ['o','o','o','o','o']
                self.GridStatus = ['b','b','b','b','b','b','b','b','b']
                return
            
            cur_pose = self.current_poses[side]
            
            dist_x = math.fabs(cur_pose.pose.position.x-x)
            dist_y = math.fabs(cur_pose.pose.position.y-y)
            dist_z = math.fabs(cur_pose.pose.position.z-z)
            if counter > 50:
                return
            counter = counter +1
            rospy.sleep(0.1)
        #print "Postion Reached", dist_x, dist_y, dist_z
        #print target_pose
    
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
        
        self.gripper_control('left', 'calibrate')
        self.gripper_control('left', 'open')
        self.gripper_control('right', 'calibrate')
        self.gripper_control('right', 'open')
        rospy.sleep(1)
        print "Game Init Starts"
        #self.move_arm('left', [0.0, 0.6, 0.2, 0.0, 1.0, 0.0, 0.0])
        #self.move_arm('left', self.LeftArmInitPose)
        joints_left_list = {'left_s0':1.6977, 'left_s1':0.1446, 'left_e0':-1.5858, 'left_e1':2.4106, 'left_w0':1.6409, 'left_w1':1.6409, 'left_w2':0.1472}
        #self.arms['left'].set_joint_position_speed(0.3)
        self.arms['left'].move_to_joint_positions(joints_left_list)
        joints_left_list = {'left_s0':0.3, 'left_s1':0.1446, 'left_e0':-1.5858, 'left_e1':2.4106, 'left_w0':1.6409, 'left_w1':1.6409, 'left_w2':0.1472}
        #self.arms['left'].move_to_joint_positions(joints_left_list)
        #rospy.sleep(2)
        #self.move_arm('right', [0.0, -0.6, 0.2, 0.0, 1.0, 0.0, 0.0])
        #self.move_arm('right', self.RightArmInitPose)
        joints_right_list = {'right_s0':-1.6977, 'right_s1':0.1446, 'right_e0':1.5858, 'right_e1':2.4106, 'right_w0':-1.6409, 'right_w1':1.6574, 'right_w2':-0.1472}
        #self.arms['right'].set_joint_position_speed(0.5)
        self.arms['right'].move_to_joint_positions(joints_right_list)
        joints_right_list = {'right_s0':-0.3, 'right_s1':0.1446, 'right_e0':1.5858, 'right_e1':2.4106, 'right_w0':-1.6409, 'right_w1':1.6574, 'right_w2':-0.1472}
        #self.arms['right'].move_to_joint_positions(joints_right_list)
        rospy.sleep(2)
        
        msg_string = 'game_engine:o:quit_session'
        self.GridStatusPub.publish(msg_string)
        
        self.GridCenter = self.GridLocations[4]
        
        # Up to here, the center of the Grid Pattern is found.
        
    def check_one_grid(self, side, grid_id):
        
        print "Checking Grid Status of grid id : ", grid_id
        center_pose = self.GridLocations[4]
        x = center_pose[0]
        y = center_pose[1]
        z = center_pose[2]
        ox = center_pose[3]
        oy = center_pose[4]
        oz = center_pose[5]
        ow = center_pose[6]
        
        new_pose = list(self.GridLocations[grid_id])
        
        new_pose[2] = new_pose[2] + 0.17
        
        self.move_arm(side, new_pose)
        
        rospy.sleep(2)
        
        msg_string = side+':check1:'+str(grid_id)
        print "Check Grid1 Cmd: ", msg_string
        self.VisionCmdPub.publish(msg_string)
        
        cv_reply = self.get_vision_reply()
    
        reply_cmd, grid_status, xy_list = self.interpret_grid_checking_reply1(cv_reply)
        
        return grid_status, xy_list
    
    def pick_from_xy(self, side, pose):
        
        x = pose[0]
        y = pose[1]
        z = pose[2]
        ox = 0.0 #pose[3]
        oy = 1.0 # pose[4]
        oz = 0.0 #pose[5]
        ow = 0.0 #pose[6]
        
        pose_list = [x, y, z + 0.2, ox, oy, oz, ow]
        self.move_arm(side, pose_list)
        rospy.sleep(1)
        self.gripper_control(side, 'open')
        pose_list2 = [x, y, z + 0.05, ox, oy, oz, ow]
        self.move_arm(side, pose_list2)
        pose_list1 = [x, y, z + 0.015 , ox, oy, oz, ow]
        self.move_arm(side, pose_list1)
        rospy.sleep(1)
        self.gripper_control(side, 'close')
        rospy.sleep(0.5)
        self.move_arm(side, pose_list)
        
        return
        
        
    def place_to_xy(self, side, pose):
        
        x = pose[0]
        y = pose[1]
        z = pose[2]
        ox = 0.0 #pose[3]
        oy = 1.0 #pose[4]
        oz = 0.0 #pose[5]
        ow = 0.0 #pose[6]
        
        pose_list = [x, y, z + 0.2, ox, oy, oz, ow]
        self.move_arm(side, pose_list)
        rospy.sleep(1)
        
        pose_list2 = [x, y, z + 0.05, ox, oy, oz, ow]
        self.move_arm(side, pose_list2)
        
        pose_list1 = [x, y, z +0.03 , ox, oy, oz, ow]
        self.move_arm(side, pose_list1)
        rospy.sleep(1)
        
        self.gripper_control(side, 'open')
        rospy.sleep(1)
        
        #pose_list2 = [x+0.01, y, z + 0.03, ox, oy, oz, ow]
        #self.move_arm(side, pose_list2)
        #rospy.sleep(2)
        
        self.move_arm(side, pose_list)
        #rospy.sleep(2)
        
        
        
        
        
        
        
    # Place all the blocks in the grids back to start position    
    # Assume a block can be in the grids or already at the start positions
    # will check each grid first, pick away one type('x'/'o') with one arm, then pick 
    # all the other type with the other arm
    def place_all_blocks(self):
        
        grid_ids = [0, 3, 6, 1, 4, 7, 2, 5, 8]
        grid_status_list = []
        xy_list = []
        for id in grid_ids:
            
            grid_status, xy = self.check_one_grid('left', id)
            print "Grid is : ", grid_status
            print "Item xy: ", xy
            grid_status_list.append(grid_status[0])
            xy_list.append(xy)
            
        print "Ready to clean the table..."
        print grid_status_list
        print xy_list
        counter = 0
        for item in grid_status_list:
            
            self.wait_for_estop()
            cur_index = counter #grid_status_list.index(item)
            grid_id = grid_ids[cur_index]
            print "Process item in grid: ", grid_id
            xy = xy_list[cur_index]
            if xy[0]>10 or xy[1]>10:
                counter = counter + 1
                continue
            
            if item == 'x':
                
                self.move_arm('right', self.RightArmInitPose)
                self.pick_from_xy('left', self.GridForRightArm[grid_id])
                x1 = self.GridForRightArm1[6][0]
                y1 = self.GridForRightArm1[6][1]+0.16
                left_slot = [x1, y1, -0.0740, 0.0, 1.0, 0.0111, 0.0]
                self.place_to_xy('left', left_slot)
                
            elif item == 'o':
                
                self.move_arm('left', self.LeftArmInitPose)
                x1 = self.GridLocations[0][0]
                y1 = self.GridLocations[0][1]-0.16
                right_slot = [x1, y1, -0.0740, 0.0, 1.0, 0.0111, 0.0]
                self.pick_from_xy('right', self.GridLocations[grid_id])
                self.place_to_xy('right', right_slot)
                
            elif item == 'b':
                
                pass
                
            counter = counter + 1
        
        
            
        return    
    
    def pick_one_grid(self, side):
        
        
        
        
        return
    
          
    def check_grid(self, side):
        
        print "Checking Grid Status..."
        center_pose = self.GridLocations[4]
        x = center_pose[0]
        y = center_pose[1]
        z = center_pose[2]
        ox = center_pose[3]
        oy = center_pose[4]
        oz = center_pose[5]
        ow = center_pose[6]
        
        pose_list1 = [x-0.02, y, z+0.12, ox, oy, oz, ow]
        pose_list2 = [x+0.1, y, z+0.12, ox, oy, oz, ow]
        pose_list3 = [x+0.22, y, z+0.12, ox, oy, oz, ow]
        poses = []
        #poses.append(pose_list1)
        #poses.append(pose_list2)
        #poses.append(pose_list3)
        for item in self.GridLocations:
            
            new_list = list(item)
            new_list[2] = z + 0.14
            poses.append(new_list)
        
        
        roi_order_list = [0, 3, 6, 1, 4, 7, 2, 5, 8]
        
        grid_final_status = []
        for pose in poses:
            self.move_arm(side, pose)
            rospy.sleep(0.5)
            grid_id = roi_order_list[poses.index(pose)]
            msg_string = side+':check:'+str(grid_id)
            print "Check Grid Cmd: ", msg_string
            self.VisionCmdPub.publish(msg_string)
            
            cv_reply = self.get_vision_reply()
        
            reply_cmd, grid_status = self.interpret_grid_checking_reply(cv_reply)
            grid_final_status.extend(grid_status)
            print "Grid Status: ", grid_status
            
        
        if len(grid_final_status) == 9:
            return grid_final_status
        else:
            return ''
            #send the grid status to game engine

        
    def pick_item(self, side, target_name):
        
        self.move_arm(side, [self.GridCenter[0], self.GridCenter[1]+0.3, self.TableZ+0.10, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(2)
        msg_string = side+':detect:'+ target_name
        self.VisionCmdPub.publish(msg_string)
        
        cv_reply = self.get_vision_reply()
        print "Grid Tracking result: ", cv_reply
        item_name, item_pose = self.interpret_vision_reply(cv_reply)
        cur_pose = self.current_poses[side]
        self.gripper_control('left', 'open')
        self.move_arm(side, [item_pose[0], item_pose[1], self.TableZ+self.BlockHeight+0.02, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(2)
        
        
        '''
        msg_string = side+':fine_tune:'+ target_name
        print "Fine tune Cmd: ", msg_string
        self.VisionCmdPub.publish(msg_string)
        cv_reply = self.get_vision_reply()
        print "Fine tune Reply: ", cv_reply
        item_name, item_pose = self.interpret_vision_reply(cv_reply)
        dx = item_pose[0]
        dy = item_pose[1]
        angle = item_pose[2]
        print "Fine tune parameters: ", dx, dy, angle
        sign_x = 1.0
        sign_y = 1.0
        while (math.fabs(dx)>15 or math.fabs(dy)>15) and not rospy.is_shutdown():
            
            cur_pose = self.current_poses[side]
            x = cur_pose.pose.position.x
            y = cur_pose.pose.position.y
            z = cur_pose.pose.position.z
            ox = cur_pose.pose.orientation.x
            oy = cur_pose.pose.orientation.y
            oz = cur_pose.pose.orientation.z
            ow = cur_pose.pose.orientation.w
            if math.fabs(dy)>15:
                
                x1 = x+0.004*sign_x
            else:
                x1 = x
            if math.fabs(dx)>15:
                
                y1 = y+0.004*sign_y
            else:
                y1 = y
                
            self.move_arm(side, [x1, y1, z, ox, oy, oz, ow])
            print "Fine Tuning Moving..."
            rospy.sleep(0.2)
            msg_string = side+':fine_tune:'+ target_name
            self.VisionCmdPub.publish(msg_string)
            cv_reply = self.get_vision_reply()
            item_name, item_pose = self.interpret_vision_reply(cv_reply)
            dx1 = item_pose[0]
            dy1 = item_pose[1]
            angle1 = item_pose[2]
            if math.fabs(dx)>math.fabs(dx1):
                sign_x = sign_x * -1.0
            
            if math.fabs(dy)>math.fabs(dy1):
                sign_x = sign_x * -1.0
                
            dx = dx1
            dy = dy1
            angle = angle1
            
            print "Fine Tuning Parameters: ", dx, dy, angle
            rospy.sleep(2)
            
        print "Fine Tuning Is Done..."  
        '''
        
        init_pose = self.current_poses[side]
        x = init_pose.pose.position.x
        y = init_pose.pose.position.y
        z = init_pose.pose.position.z
        ox = init_pose.pose.orientation.x
        oy = init_pose.pose.orientation.y
        oz = init_pose.pose.orientation.z
        ow = init_pose.pose.orientation.w
        
        self.move_arm(side, [x, y, self.TableZ+self.BlockHeight-0.01, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(2)
        self.gripper_control('left', 'close')
        rospy.sleep(1)
        self.move_arm(side, [x, y, self.TableZ+self.BlockHeight+0.06, 0.0, 1.0, 0.0, 0.0])
        
        #self.move_arm(side, [item_pose[0], item_pose[1], self.TableHeight+self.GripperLength+0.05, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(2)
        #self.gripper_control('left', 'close')
    
    def place_item(self, side, col, row):
        
        init_pose = self.current_poses['left']
        x = init_pose.pose.position.x
        y = init_pose.pose.position.y
        z = init_pose.pose.position.z
        ox = init_pose.pose.orientation.x
        oy = init_pose.pose.orientation.y
        oz = init_pose.pose.orientation.z
        ow = init_pose.pose.orientation.w
        
        if col>2 or col<0 or row>2 or row<0:
            return
        print "about to place..."
        
        self.move_arm(side, [x, y, self.TableZ+self.BlockHeight+0.2, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(1.5)
        
        pose_list = self.GridLocations[col + row*3]
        pose_list1 = list(pose_list)
        pose_list1[2] = pose_list1[2]+0.15
        self.move_arm(side, pose_list1)
        rospy.sleep(1.5)
        
        pose_list1 = list(pose_list)
        pose_list1[2] = pose_list1[2]+0.025
        self.move_arm(side, pose_list1)
        rospy.sleep(1.5)
        self.gripper_control(side, 'open')
        pose_list1 = list(pose_list)
        pose_list1[2] = pose_list1[2]+0.15
        self.move_arm(side, pose_list1)
        
        
    def load_location_data(self):
        self.GridLocations = []
        self.TableZ = 0.0
        file = open("./src/phm/grids_location.txt", "r")
        for i in range(0, 9):
            line = file.readline()
            temp_list = []
            for j in line.split():
                temp_list.append(float(j))
            self.GridLocations.append(temp_list)
            #print temp_list
            self.TableZ = self.TableZ + temp_list[2]
        self.TableZ = self.TableZ / 9
        file.close()
        print self.GridLocations
        print "Table Z Value:", self.TableZ
        
        self.GridRoiLocations = []
        self.GridRoiPose = []
        file = open("./src/phm/grids_roi_location.txt", "r")
        for i in range(0, 11):
            line = file.readline()
            temp_list = []
            for j in line.split():
                if i<9:
                    temp_list.append(int(j))
                elif i==9:
                    temp_list.append(float(j))
            
            if i<9:
                self.GridRoiLocations.append(temp_list)
            elif i==9:
                self.GridRoiPose = temp_list
        
        file.close()
        print self.GridRoiLocations
        print self.GridRoiPose

    def start_roi_recording(self):
        pass
        
    def pick_test(self, col, row):
        
        self.gripper_control('left', 'calibrate')
        self.gripper_control('left', 'open')
        rospy.sleep(1)
        init_pose = self.current_poses['left']
        x = init_pose.pose.position.x
        y = init_pose.pose.position.y
        z = init_pose.pose.position.z
        ox = init_pose.pose.orientation.x
        oy = init_pose.pose.orientation.y
        oz = init_pose.pose.orientation.z
        ow = init_pose.pose.orientation.w
        
        if col>2 or col<0 or row>2 or row<0:
            return
        print "about to pick"
        pose_list = self.GridLocations[col + row*3]
        print pose_list
        pose_list1 = list(pose_list)
        pose_list1[2] = pose_list1[2]+0.1
        self.move_arm('left', pose_list)
        rospy.sleep(1)
        return
        self.move_arm('left', pose_list)
        rospy.sleep(1)
        self.gripper_control('left', 'close')
        
        self.move_arm('left', pose_list1)
        rospy.sleep(1)
        init_pose_list = [x, y, z, ox, oy, oz, ow]
        self.move_arm('left', init_pose_list)
        rospy.sleep(1)
    
    def next_move_callback(self, msg):
        
        self.NextMoveReply = msg.data
    
    
    def wait_for_next_move(self):
        
        print "Wait For Next Move Reply..."
        next_move_reply = self.NextMoveReply
        
        while next_move_reply == '':
            
            if self.GameState == 'Estop_on':
                return 'Estop_on'
            
            next_move_reply = self.NextMoveReply
            rospy.sleep(0.1)
            
        self.NextMoveReply = ''
        
        return next_move_reply
        
    def interpret_next_move(self, msg_string):
        if msg_string == '':
            rospy.logerr("Next Move Reply Failure: msg is empty")
        
        msg_segments = msg_string.split()
        
        if len(msg_segments)!= 2:
            rospy.logerr("Next Move Reply Failure: msg segment not correct")
            return '', -1
            
        item = msg_segments[0]
        id = int(msg_segments[1]) # grid id: 0~8
        if item in ['x', 'o']:
            return item, id
        elif item == 'draw':
            return 'draw', -1
        elif item == 'win':
            return 'win', id
        else:
            return '', -1
    
    def find_in_slots(self, side, item):
        
        if side == 'left':
            
            for item in self.LeftSlots:
                
                if item != 'b':
                    return self.LeftSlots.index(item)
                
        elif side == 'right':
            
            for item in self.RightSlots:
                
                if item != 'b':
                    return self.RightSlots.index(item)
                
        return -1
                
    def get_grid_status_string(self):
        
        return_string = ''
        for item in self.GridStatus:
            return_string = return_string + item + ' '
            
        return return_string

    def find_empty_slot(self, item):
        
        counter = 0
        if item == 'x':
            for item in self.LeftSlots:
                if self.LeftSlots[counter] == 'b':
                    return counter
                counter = counter + 1
        
        elif item == 'o':
            
            for item in self.RightSlots:
                if self.RightSlots[counter] == 'b':
                    return counter
                
                counter = counter + 1
                
        return -1
    
    def place_all_blocks1(self):
        
        #self.LeftSlots = ['x','x','x','x','b']
        #self.RightSlots = ['b','o','o','o','o']
        #self.GridStatus = ['b','b','b','b','b','b','b','b','x']
        
        print "Reset the Table..."
        grid_ids = [0, 3, 6, 1, 4, 7, 2, 5, 8]
        grid_status_list = []
        xy_list = []
        
        counter = 0
        for item in self.GridStatus:
            
            if item == 'x':
                
                self.move_arm('right', self.RightArmInitPose)
                self.pick_from_xy('left', self.GridForLeftArm[counter])
                
                slot_id = self.find_empty_slot('x')
                self.place_to_xy('left', self.LeftSlotsLocation1[slot_id])
                self.LeftSlots[slot_id] = 'x'
                self.GridStatus[counter] = 'b'
                
            elif item == 'o':
                
                self.move_arm('left', self.LeftArmInitPose)
                
                self.pick_from_xy('right', self.GridForRightArm[counter])
                
                slot_id = self.find_empty_slot('o')
                self.place_to_xy('right', self.RightSlotsLocation1[slot_id])
                
                self.RightSlots[slot_id] = 'o'
                self.GridStatus[counter] = 'b'
                
            elif item == 'b':
                
                pass
                
            counter = counter + 1
        
        
            
        return
    
    def move_to_init(self, side):
        
        if side == 'left':
            joints_left_list = {'left_s0':1.6977, 'left_s1':0.1446, 'left_e0':-1.5858, 'left_e1':2.4106, 'left_w0':1.6409, 'left_w1':1.6409, 'left_w2':0.1472}
        
            self.arms['left'].move_to_joint_positions(joints_left_list)
        #joints_left_list = {'left_s0':0.3, 'left_s1':0.1446, 'left_e0':-1.5858, 'left_e1':2.4106, 'left_w0':1.6409, 'left_w1':1.6409, 'left_w2':0.1472}
        elif side == 'right':
            joints_right_list = {'right_s0':-1.6977, 'right_s1':0.1446, 'right_e0':1.5858, 'right_e1':2.4106, 'right_w0':-1.6409, 'right_w1':1.6574, 'right_w2':-0.1472}
        
            self.arms['right'].move_to_joint_positions(joints_right_list)
        
        return
    
    
    def demo_play(self):
        
        
        
        print "Entering Demo Mode..."
        
        self.LeftSlots = ['x','x','x','x','x']
        self.RightSlots = ['o','o','o','o','o']
        self.GridStatus = ['b','b','b','b','b','b','b','b','b']
        
        
        while self.GameState != 'Estop_on':
            
            first_play_id = randint(0,1)
            first_grid_id = randint(0,8)
            
            msg_string = ''
            next_move = ''
            if first_play_id == 0: # 'o' placed first
                
                print "First Place is 'o' at %d" % first_grid_id
                self.FirstPlayMarker = 'o'
                msg_string = 'game_engine:o:start'
                self.GridStatus[first_grid_id] = 'o'
                self.GridStatusPub.publish(msg_string)
                msg_string = 'game_status:o:' + self.get_grid_status_string()
                slot_id = self.find_in_slots('right', 'o')
                
                if slot_id in range(0, 5):
                    
                    self.pick_from_xy('right', self.RightSlotsLocation[slot_id])
                    self.place_to_xy('right', self.GridForRightArm1[first_grid_id])
                    
                    self.move_arm('right', self.RightArmInitPose)
                    #self.move_to_init('right')
                    
                self.RightSlots[slot_id] = 'b'
                
            else: # 'x' placed first
                
                print "First Place is 'x' at %d" % first_grid_id
                
                self.FirstPlayMarker = 'x'
                msg_string = 'game_engine:x:start'
                self.GridStatus[first_grid_id] = 'x'
                self.GridStatusPub.publish(msg_string)
                msg_string = 'game_status:x:' + self.get_grid_status_string()
                
                slot_id = self.find_in_slots('left', 'x')
                if slot_id in range(0, 6):
                    
                    self.pick_from_xy('left', self.LeftSlotsLocation[slot_id])
                    self.place_to_xy('left', self.GridForLeftArm1[first_grid_id])
                    
                    self.move_arm('left', self.LeftArmInitPose)
                    #self.move_to_init('left')
                self.LeftSlots[slot_id] = 'b'
            self.GridStatusPub.publish(msg_string)
            
            sessionDone = False
            while not sessionDone:
            
                next_move = self.wait_for_next_move()
                
                if next_move == 'Estop_on':
                    
                    self.LeftSlots = ['x','x','x','x','x']
                    self.RightSlots = ['o','o','o','o','o']
                    self.GridStatus = ['b','b','b','b','b','b','b','b','b']
                    msg_string = 'game_engine:o:quit_session'
                    self.GridStatusPub.publish(msg_string)
                    
                    return
                
                item, id = self.interpret_next_move(next_move)
                print "Item: ", item, "Id: ", id
                
                if item in ['x'] and id in range(0, 9):
                    
                    slot_id = self.find_in_slots('left', 'x')
                    print "Pick x from left slot: ", self.LeftSlots[slot_id]
                    print "place it to: ", self.GridForLeftArm[id]
                    self.pick_from_xy('left', self.LeftSlotsLocation[slot_id])
                    self.place_to_xy('left', self.GridForLeftArm1[id])
                    self.move_arm('left', self.LeftArmInitPose)
                    print "Left Arm Back to Init Position..."
                    #self.move_to_init('left')
                    self.LeftSlots[slot_id] = 'b'
                    self.GridStatus[id] = item
                    msg_string = 'game_status:x:' + self.get_grid_status_string()
                    self.GridStatusPub.publish(msg_string)
                    
                elif item in ['o'] and id in range(0, 9):
                    slot_id = self.find_in_slots('right', 'o')
                    print "Pick o from right slot: ", self.LeftSlots[slot_id]
                    print "place it to: ", self.GridForRightArm[id]
                    self.pick_from_xy('right', self.RightSlotsLocation[slot_id])
                    self.place_to_xy('right', self.GridForRightArm1[id])
                    self.move_arm('right', self.RightArmInitPose)
                    print "Right Arm Back to Init Position..."
                    #self.move_to_init('right')
                    self.GridStatus[id] = item
                    self.RightSlots[slot_id] = 'b'
                    msg_string = 'game_status:o:' + self.get_grid_status_string()
                    self.GridStatusPub.publish(msg_string)
                
                elif item == 'draw':
                    sessionDone = True
                    self.GameStatus = 'Done'
                    break
                    
                elif item == 'win':
                    sessionDone = True
                    self.GameStatus = 'Done'
                    break
            
                    
            
            self.place_all_blocks1()
            
            
            
            
        
            
    
        
        
    
    def run(self):
        
        self.GameState = 'NoInit'
        print "Loading Game Data..."
        self.load_location_data()
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        rs.enable()
        
        #self.pick_test(1,0)
        print "Wait For Button"
        self.wait_button_on(0.1, 100) # User has to keep pressing the button for at least 1 second
        
        ##self.pck_item('left', 'x')
        ##self.place_item('left', 1, 1)
        #self.check_grid('left')
        #grid_status = self.check_grid('left')
        #print "Grid Status: ", grid_status
        #self.place_all_blocks()
        grid_xy = [[0,0], [1,0], [2,0], [0,1], [1,1], [2,1], [0,2], [1,2], [2,2]]
        counter = 0
        
        

        #msg_string = 'game_status:x:'
        
        #for item in grid_status:
        #    msg_string = msg_string+item + ' '
            
        #self.GridStatusPub.publish(msg_string)
        
        while not rospy.is_shutdown():
            
            
            if self.GameState == 'NoInit':
                
                self.init_game()
                print "Game Initilized"
                self.GameState = 'Init'
                
                
            
            
            elif self.GameState == 'Init':
                
                if 'o' in self.GridStatus or 'x' in self.GridStatus:
                    
                    self.place_all_blocks()
                self.GameState = 'InitDone'
            
            elif self.GameState == 'Done':
                
                self.GameState = 'InitDone'
                rospy.sleep(5)
                
            elif self.GameState == 'InitDone':
                
                self.demo_play()
                
            elif self.GameState == 'Estop_on':
                
                print "E Stop is on"
                self.wait_for_estop()
                pass
                
            elif self.GameState == 'Estop_reset':
                
                print "Press Button to Re enalbe robot"
                self.wait_button_on(0.2, 200000)
                rs = baxter_interface.RobotEnable(CHECK_VERSION)
                restart_status = False
                while not restart_status:
                    try:
                        rs.enable()
                        restart_status = True
                    except Exception, e:
                        rospy.logerr(e.strerror)
                        restart_status = False
                    rospy.sleep(0.1)
                
                print "Task Restart"
                self.GameState = 'NoInit'
                
                
            
            
                
                
                
                
                
                
            
            '''
            item, id = self.interpret_next_move(self.wait_for_next_move())
            if item == '':
                rospy.sleep(0.1)
                continue
            #elif id>9:
                
            self.pick_item('left', item)
            col = grid_xy[id][0]
            row = grid_xy[id][1]
            self.place_item('left', col, row)
            rospy.sleep(4)
            grid_status = self.check_grid('left')
            
            msg_string = 'game_status:x:'
        
            for item in grid_status:
                msg_string = msg_string+item + ' '
                
            self.GridStatusPub.publish(msg_string)
            
            self.wait_button_on(0.2)
            '''
            
            '''
            grid_status = self.check_grid('left')
            print "previous grid status: ", grid_status
            print "Current grid status: ", grid_status1
            if grid_status1 == grid_status:
                print "Grid Status not Changed..."
                rospy.sleep(0.1)
                continue
            
            counter = 0
            for grid in grid_status1:
                
                if grid == 'b':
                    self.pick_item('left', 'x')
                    col = grid_xy[counter][0]
                    row = grid_xy[counter][1]
                    self.place_item('left', col, row)
                    
                    break
                
                counter = counter + 1
            
            grid_status = list(grid_status1)
            '''
            
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
    
    
    
    