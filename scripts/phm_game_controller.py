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
        self.GridStatusPub = rospy.Publisher('game_engine_cmd', String, queue_size=10)
        self.VisionReply = ''
        self.ArmReply = ''
        self.NextMoveReply = ''
        rospy.init_node("game_controller")
        rospy.Subscriber('arm_reply', String, self.arms_reply_callback)
        rospy.Subscriber('vision_reply', String, self.vision_reply_callback)
        rospy.Subscriber('next_move', String, self.next_move_callback)
        rospy.Subscriber('next_move', String, self.next_move_callback)
        
        self.GridCenter = []
        self.TableHeight = 0.0
        self.GripperLength = 0.125
        self.LeftArmInitPose = [0.5, 0.6, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.RightArmInitPose = [0.5, -0.6, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.current_poses = None
        self.GridLocations = []
        self.GridRoiLocations = []
        self.GridRoiPose = []
        self.TableZ = 100.0
        self.BlockHeight = 0.03
        
        
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        left = baxter_interface.Gripper('left', CHECK_VERSION)
        right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.baxter_grippers = {'left':left, 'right':right}
        
    def get_vision_reply(self):
        
        cv_reply = self.VisionReply
        
        while cv_reply == '':
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
        
    def gripper_control(self, side, action):
        gripper = self.baxter_grippers[side]
        if action == 'calibrate':
            gripper.calibrate()
        elif action == 'open':
            gripper.open()
        elif action == 'close':
            gripper.close()
        
        
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
        #print self.current_poses
        cur_pose = self.current_poses[side]
        dist_x = math.fabs(cur_pose.pose.position.x-x)
        dist_y = math.fabs(cur_pose.pose.position.y-y)
        dist_z = math.fabs(cur_pose.pose.position.z-z)
        
        while dist_x>0.005 or dist_y>0.005 or dist_z>0.005:
            cur_pose = self.current_poses[side]
            
            dist_x = math.fabs(cur_pose.pose.position.x-x)
            dist_y = math.fabs(cur_pose.pose.position.y-y)
            dist_z = math.fabs(cur_pose.pose.position.z-z)
            rospy.sleep(0.1)
        print "Postion Reached", dist_x, dist_y, dist_z
        print target_pose
    
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
        rospy.sleep(1)
        print "Game Init Starts"
        self.move_arm('left', self.LeftArmInitPose)
        rospy.sleep(2)
        self.move_arm('right', self.RightArmInitPose)
        rospy.sleep(2)
        
        self.GridCenter = self.GridLocations[4]
        
        # Up to here, the center of the Grid Pattern is found.
          
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
            new_list[2] = z + 0.13
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
        else:
            return '', -1
            
    
    def run(self):
        
        
        print "Loading Game Data..."
        self.load_location_data()
        #self.pick_test(1,0)
        
        self.init_game()
        print "Game Initilized"
        
        ##self.pick_item('left', 'x')
        ##self.place_item('left', 1, 1)
        #self.check_grid('left')
        grid_status = self.check_grid('left')
        print "Grid Status: ", grid_status
        grid_xy = [[0,0], [1,0], [2,0], [0,1], [1,1], [2,1], [0,2], [1,2], [2,2]]
        counter = 0

        msg_string = 'game_status:x:'
        
        for item in grid_status:
            msg_string = msg_string+item + ' '
            
        self.GridStatusPub.publish(msg_string)
        
        while not rospy.is_shutdown():
            
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
    
    
    
    