#!/usr/bin/env python

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

import argparse
import struct
import sys

import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
import baxter_interface

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from baxter_core_msgs.msg import EndpointState
import message_filters
from std_msgs.msg import String
import threading

class MoveArms(object):
    
    def __init__(self):
    
        self.current_poses = None
        self.init_pose = None
        self.current_arm_cmd = ''
        self.cmd_threadlock = threading.Lock()
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        ts = message_filters.ApproximateTimeSynchronizer([left_arm_msg, right_arm_msg], 10, 0.05)
        ts.registerCallback(self._pose_callback)
        
        rospy.Subscriber('robot_arms_cmd', String, self._arms_cmd)
        
        left_arm = baxter_interface.Limb("left")
        right_arm = baxter_interface.Limb("right")
        self.arms = {'left':left_arm, 'right':right_arm}

    def _arms_cmd(self, msg):
        cmd_string = msg.data
        print "\nReceiving Arms Command\n", cmd_string
        with self.cmd_threadlock:
            self.current_arm_cmd = cmd_string
        #rospy.sleep(0.05)
        
        
    
    def arms_cmd_parser(self, msg):
        msg_segment = msg.split(':')
        if len(msg_segment) != 3:
            return None, None, None
            
        arm = msg_segment[0]
        cmd = msg_segment[1]
        
        pose = [float(m) for m in msg_segment[2].split(',')]
        
        return arm, cmd, pose
    
    
    # a wrapper of PoseStamped()
    # pose_list: [x, y, z, ox, oy, oz, ow]
    # header: from Header(stamp, frame_id)
    # return: a completed PoseStamped()
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
        
    def get_pose(self):
    
        return self.current_pose
        
    
    # single direction endpoint moving
    # disp: displacement in meters (+/- float number)
    # arm: 'left' or 'right' (string)
    # return: 0 if can't make the move, 1 if can make the move
    def move_distance(self, disp, arm):
    
        
        cur_pose = self.current_poses[arm]

        new_pose = self.make_pose_stamp([cur_pose.pose.position.x+disp[0], \
                                         cur_pose.pose.position.y+disp[1], \
                                         cur_pose.pose.position.z+disp[2], \
                                         cur_pose.pose.orientation.x, \
                                         cur_pose.pose.orientation.y, \
                                         cur_pose.pose.orientation.z, \
                                         cur_pose.pose.orientation.w], \
                                         Header(stamp=rospy.Time.now(), frame_id='base'))
           
        ns = "ExternalTools/" + arm + "/PositionKinematicsNode/IKService"
        print ns
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        
        ikreq.pose_stamp.append(new_pose)
        
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            #rospy.logerr("Service call failed: %s" % (e,))
            return 0
            
        limb_joints = None    
        if (resp.isValid[0]):
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print "\nFinish IK moving\n"
            
        else:
            return 0
        
        
        self.arms['left'].move_to_joint_positions(limb_joints)
        
        return 1
    
    # single axis ratation
    # angle: angle
    def turn_angle(self, angle, axis):
        
        cur_pose = self.current_poses[arm]
        
        new_pose = self.make_pose_stamp([cur_pose.pose.position.x, \
                                         cur_pose.pose.position.y, \
                                         cur_pose.pose.position.z, \
                                         cur_pose.pose.orientation.x, \
                                         cur_pose.pose.orientation.y, \
                                         cur_pose.pose.orientation.z, \
                                         cur_pose.pose.orientation.w], \
                                         Header(stamp=rospy.Time.now(), frame_id='base'))
    
        return
        

    def move_to(self, point):
    
        return
        
        
    def run(self):
    
        while not rospy.is_shutdown():
        
            msg_string = ''
            
            with self.cmd_threadlock:
                msg_string = self.current_arm_cmd
                self.current_arm_cmd = ''
                #rospy.sleep(0.05)
                
            if msg_string != '':
                arm, cmd, pose_list = self.arms_cmd_parser(msg_string)
                msg_string = ''
                if (arm != None):
                    print pose_list[0:3]
                    self.move_distance(pose_list[0:3], arm)
                    pass
                    
           
                
        
            rospy.sleep(0.1)
    


def main():
    rospy.init_node("phm_ik_service_client")
    mr = MoveArms()
    #mr.move_distance
    mr.run()
    
    
    
if __name__ == '__main__':
    sys.exit(main())
    
    
    

