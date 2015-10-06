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

class MoveArms(object):
    
    def __init__(self):
    
        self.current_poses = None
        self.init_pose = None
        self.current_arm_cmd = ''
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
        self.current_arm_cmd = cmd_string
        
        
    
    def arms_cmd_parser(self, msg):
        msg_segment = msg.split(':')
        if len(msg_segment) != 3:
            return None, None, None
            
        arm = msg_segment[0]
        cmd = msg_segment[1]
        
        pose = [float(m) for m in msg_segment[2].split(',')]
        
        return arm, cmd, pose
    
        
    
    def _pose_callback(self, left_msg, right_msg):
    
        pose1 = left_msg.pose
        x1 = pose1.position.x
        y1 = pose1.position.y
        z1 = pose1.position.z
        
        ox1 = pose1.orientation.x
        oy1 = pose1.orientation.y
        oz1 = pose1.orientation.z
        ow1 = pose1.orientation.w
        
        pose2 = right_msg.pose
        x2 = pose2.position.x
        y2 = pose2.position.y
        z2 = pose2.position.z
        
        ox2 = pose2.orientation.x
        oy2 = pose2.orientation.y
        oz2 = pose2.orientation.z
        ow2 = pose2.orientation.w
        
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        cp1 = PoseStamped()
        cp1.header = header
        cp1.pose.position=Point(x=x1, y=y1, z=z1,)
        cp1.pose.orientation=Quaternion(x=ox1, y=oy1, z=oz1, w=ow1,)
        
        cp2 = PoseStamped()
        cp2.header = header
        cp2.pose.position=Point(x=x2, y=y2, z=z2,)
        cp2.pose.orientation=Quaternion(x=ox2, y=oy2, z=oz2, w=ow2,)
        
        self.current_poses = {'left':cp1, 'right':cp2}
        
        
        #print "\nleft\n", cp1, "\n\nright\n", cp2
            
        return
        
    def get_pose(self):
    
        return self.current_pose
        
    
    # single direction endpoint moving
    # disp: displacement in meters (+/- float number)
    # arm: 'left' or 'right' (string)
    # return: 0 if can't make the move, 1 if can make the move
    def move_distance(self, disp, arm):
    
        
        cur_pose = self.current_poses[arm]
  
        x1 = cur_pose.pose.position.x
        y1 = cur_pose.pose.position.y
        z1 = cur_pose.pose.position.z
        
        x2 = x1 + disp[0]
        y2 = y1 + disp[1]
        z2 = z1 + disp[2]
        
        ox2 = cur_pose.pose.orientation.x
        oy2 = cur_pose.pose.orientation.y
        oz2 = cur_pose.pose.orientation.z
        ow2 = cur_pose.pose.orientation.w
        
        new_pose = PoseStamped()
        new_pose.header = Header(stamp=rospy.Time.now(), frame_id='base')
        new_pose.pose.position=Point(x=x2, y=y2, z=z2,)
        new_pose.pose.orientation=Quaternion(x=ox2, y=oy2, z=oz2, w=ow2,)
           
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
    
        return
        

    def move_to(self, point):
    
        return


def main():
    rospy.init_node("phm_ik_service_client")
    mr = MoveArms()
    #mr.move_distance([0.0, 0.0, -0.1], 'left')
    
    while not rospy.is_shutdown():
        
        
        
            rospy.sleep(0.1)
    
if __name__ == '__main__':
    sys.exit(main())
    
    
    

