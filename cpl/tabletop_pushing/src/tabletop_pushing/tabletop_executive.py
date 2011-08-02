#!/usr/bin/env python
# Software License Agreement (BSD License)
#
#  Copyright (c) 2011, Georgia Institute of Technology
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#  * Neither the name of the Georgia Institute of Technology nor the names of
#     its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('tabletop_pushing')
import rospy
import hrl_pr2_lib.linear_move as lm
import hrl_pr2_lib.pr2 as pr2
import hrl_lib.tf_utils as tfu
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
from tabletop_pushing.srv import *
from math import sin, cos, pi
import sys

class TabletopExecutive:

    def __init__(self, use_fake_push_pose=False):
        # TODO: Replace these parameters with learned / perceived values
        # The offsets should be removed and learned implicitly
        self.gripper_push_dist = rospy.get_param('~gripper_push_dist',
                                                 0.15)
        self.gripper_x_offset = rospy.get_param('~gripper_push_start_x_offset',
                                                -0.03)
        self.gripper_y_offset = rospy.get_param('~gripper_push_start_x_offset',
                                                0.0)
        self.gripper_z_offset = rospy.get_param('~gripper_push_start_z_offset',
                                                0.00)

        self.gripper_sweep_dist = rospy.get_param('~gripper_sweep_dist',
                                                 0.25)
        self.sweep_x_offset = rospy.get_param('~gripper_sweep_start_x_offset',
                                              0.10)
        self.sweep_y_offset = rospy.get_param('~gripper_sweep_start_y_offset',
                                              0.05)
        self.sweep_z_offset = rospy.get_param('~gripper_sweep_start_z_offset',
                                              0.02)

        self.overhead_push_dist = rospy.get_param('~overhead_push_dist',
                                                 0.25)
        self.overhead_x_offset = rospy.get_param('~overhead_push_start_x_offset',
                                                 0.00)
        self.overhead_y_offset = rospy.get_param('~overhead_push_start_x_offset',
                                                 0.00)
        self.overhead_z_offset = rospy.get_param('~overhead_push_start_z_offset',
                                                 0.02)

        self.push_pose_proxy = rospy.ServiceProxy('get_push_pose', PushPose)
        self.gripper_push_proxy = rospy.ServiceProxy('gripper_push',
                                                     GripperPush)
        self.gripper_sweep_proxy = rospy.ServiceProxy('gripper_sweep',
                                                      GripperPush)
        self.overhead_push_proxy = rospy.ServiceProxy('overhead_push',
                                                      GripperPush)
        self.raise_and_look_push_proxy = rospy.ServiceProxy('raise_and_look',
                                                            RaiseAndLook)
        self.table_proxy = rospy.ServiceProxy('get_table_location', LocateTable)
        self.use_fake_push_pose = use_fake_push_pose

    def run(self,
            num_l_gripper_pushes, num_l_sweeps, num_l_overhead_pushes,
            num_r_gripper_pushes, num_r_sweeps, num_r_overhead_pushes):

        # TODO: Get table height and raise to that before anything else
        table_req = LocateTableRequest()
        table_req.recalculate = True
        raise_req = RaiseAndLookRequest()
        raise_req.table_centroid = self.table_proxy(table_req);
        table_res = self.raise_and_look_push_proxy(raise_req)

        for i in xrange(num_l_gripper_pushes):
            self.gripper_push_object(self.gripper_push_dist, 'l')
        for i in xrange(num_r_gripper_pushes):
            self.gripper_push_object(self.gripper_push_dist, 'r')
        for i in xrange(num_l_sweeps):
            self.sweep_object(self.gripper_sweep_dist, 'l')
        for i in xrange(num_r_sweeps):
            self.sweep_object(self.gripper_sweep_dist, 'r')
        for i in xrange(num_l_overhead_pushes):
            self.overhead_push_object(self.overhead_push_dist, 'l')
        for i in xrange(num_r_overhead_pushes):
            self.overhead_push_object(self.overhead_push_dist, 'r')

    def gripper_push_object(self, push_dist, which_arm):
        # Make push_pose service request
        pose_req = PushPoseRequest()
        pose_req.use_laser = False

        # Get push pose
        rospy.loginfo("Calling push pose service")
        try:
            if self.use_fake_push_pose:
                pose_res = PushPoseResponse()
                pose_res.push_pose.header.frame_id = '/torso_lift_link'
                pose_res.push_pose.pose.position.x = 0.7
                pose_res.push_pose.pose.position.y = 0.0
                pose_res.push_pose.pose.position.z = -0.15
            else:
                pose_res = self.push_pose_proxy(pose_req)
        except rospy.ServiceException, e:
            rospy.logwarn("Service did not process request: %s"%str(e))
            return

        # Convert pose response to correct push request format
        push_req = GripperPushRequest()
        push_req.start_point.header = pose_res.push_pose.header
        push_req.start_point.point = pose_res.push_pose.pose.position

        # TODO: Correctly set the wrist yaw
        # orientation = pose_res.push_pose.pose.orientation
        wrist_yaw = 0.0 # 0.25*pi # -0.25*pi
        push_req.wrist_yaw = wrist_yaw
        push_req.desired_push_dist = push_dist

        # TODO: Remove these offsets and incorporate them directly into the
        # perceptual inference
        # Offset pose to not hit the object immediately
        push_req.start_point.point.x += self.gripper_x_offset*cos(wrist_yaw)
        push_req.start_point.point.y += self.gripper_x_offset*sin(wrist_yaw)
        push_req.start_point.point.z += self.gripper_z_offset

        push_req.left_arm = (which_arm == 'l')
        push_req.right_arm = not push_req.left_arm

        # Call push service
        rospy.loginfo("Calling gripper push service")
        push_res = self.gripper_push_proxy(push_req)

    def sweep_object(self, push_dist, which_arm):
        # Make push_pose service request
        pose_req = PushPoseRequest()
        pose_req.use_laser = False

        # Get push pose
        rospy.loginfo("Calling push pose service")
        try:
            if self.use_fake_push_pose:
                pose_res = PushPoseResponse()
                pose_res.push_pose.header.frame_id = '/torso_lift_link'
                pose_res.push_pose.pose.position.x = 0.7
                pose_res.push_pose.pose.position.y = 0.0
                pose_res.push_pose.pose.position.z = -0.15
            else:
                pose_res = self.push_pose_proxy(pose_req)

        except rospy.ServiceException, e:
            rospy.logwarn("Service did not process request: %s"%str(e))
            return

        # Convert pose response to correct push request format
        sweep_req = GripperPushRequest()
        sweep_req.start_point.header = pose_res.push_pose.header
        sweep_req.start_point.point = pose_res.push_pose.pose.position
        # rospy.loginfo('Push pose point:' + str(sweep_req.start_point.point))

        # TODO: Correctly set the wrist yaw
        # orientation = pose_res.push_pose.pose.orientation
        wrist_yaw = 0.0 # 0.25*pi
        sweep_req.wrist_yaw = wrist_yaw
        sweep_req.desired_push_dist = push_dist

        sweep_req.left_arm = (which_arm == 'l')
        sweep_req.right_arm = not sweep_req.left_arm

        if sweep_req.left_arm:
            y_offset_dir = +1
        else:
            y_offset_dir = -1

        # TODO: Remove these offsets and incorporate them directly into the perceptual inference
        # Offset pose to not hit the object immediately
        sweep_req.start_point.point.x += (self.sweep_x_offset*cos(wrist_yaw) +
                                          self.sweep_y_offset*sin(wrist_yaw))
        sweep_req.start_point.point.y += y_offset_dir*(
            self.sweep_x_offset*sin(wrist_yaw) + self.sweep_y_offset*cos(wrist_yaw))
        sweep_req.start_point.point.z += self.sweep_z_offset

        # rospy.loginfo('Sweep start point:' + str(sweep_req.start_point.point))

        # Call push service
        rospy.loginfo("Calling gripper sweep service")
        sweep_res = self.gripper_sweep_proxy(sweep_req)

    def overhead_push_object(self, push_dist, which_arm):
        # Make push_pose service request
        pose_req = PushPoseRequest()
        pose_req.use_laser = False

        # Get push pose
        rospy.loginfo("Calling push pose service")
        try:
            if self.use_fake_push_pose:
                pose_res = PushPoseResponse()
                pose_res.push_pose.header.frame_id = '/torso_lift_link'
                pose_res.push_pose.pose.position.x = 0.7
                pose_res.push_pose.pose.position.y = 0.0
                pose_res.push_pose.pose.position.z = -0.15
            else:
                pose_res = self.push_pose_proxy(pose_req)
        except rospy.ServiceException, e:
            rospy.logwarn("Service did not process request: %s"%str(e))
            return

        # Convert pose response to correct push request format
        push_req = GripperPushRequest()
        push_req.start_point.header = pose_res.push_pose.header
        push_req.start_point.point = pose_res.push_pose.pose.position

        # TODO: Correctly set the wrist yaw
        # orientation = pose_res.push_pose.pose.orientation
        wrist_yaw = 0.0 # -0.25*pi
        push_req.wrist_yaw = wrist_yaw
        push_req.desired_push_dist = push_dist

        push_req.left_arm = (which_arm == 'l')
        push_req.right_arm = not push_req.left_arm

        if push_req.left_arm:
            y_offset_dir = +1
        else:
            y_offset_dir = -1

        # TODO: Remove these offsets and incorporate them directly into the perceptual inference
        # Offset pose to not hit the object immediately
        push_req.start_point.point.x += (self.overhead_x_offset*cos(wrist_yaw) +
                                          self.overhead_y_offset*sin(wrist_yaw))
        push_req.start_point.point.y += y_offset_dir*(
            self.overhead_x_offset*sin(wrist_yaw) +
            self.overhead_y_offset*cos(wrist_yaw))
        push_req.start_point.point.z += self.overhead_z_offset

        # rospy.loginfo('Push start point:' + str(push_req.start_point.point))

        # Call push service
        rospy.loginfo("Calling overhead push service")
        push_res = self.overhead_push_proxy(push_req)

if __name__ == '__main__':
    node = TabletopExecutive(True)
    node.run(1, 1, 1, 1, 1, 1)
