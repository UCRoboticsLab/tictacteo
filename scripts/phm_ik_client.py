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

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from baxter_core_msgs.msg import EndpointState
import message_filters

class RobotPose(object):
    
    def __init__(self):
    
        self.current_pose = None
        self.init_pose = None
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        ts = message_filters.TimeSynchronizer([left_arm_msg, right_arm_msg], 10)
        ts.registerCallback(_pose_callback)
        
        left_arm = baxter_interface.Limb("left")
        right_arm = baxter_interface.Limb("right")
        self.arms = {'left':left_arm, 'right':right_arm}

    def _pose_callback(self, left_msg, right_msg):
    
        pose = msg.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        ox = pose.orientation.x
        oy = pose.orientation.y
        oz = pose.orientation.z
        ow = pose.orientation.w
        
        
        self.current_pose = PoseStamped()
        self.current_pose.header=Header(stamp=rospy.Time.now(), frame_id='base')
        self.current_pose.pose.position=Point(x, y, z,)
        self.current_pose.pose.orientation=Quaternion(x=ox, y=oy, z=oz, w=ow,)
            
        return
        
    def get_pose(self)
    
        return self.current_pose
        
        
    def move_distance(self, relative_point):
    
        
    
        return
        
    def move_to(self, point):
    
        return


def main()
    rospy.init_node("rsdk_ik_service_client")
    
    
    

