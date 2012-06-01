#! /usr/bin/python
import roslib; roslib.load_manifest('simple_move_base')
import rospy
import tf.transformations as tr
import hrl_lib.tf_utils as tfu
import hrl_lib.util as ut
import simple_move_base.msg as hm
import tf

import geometry_msgs.msg as gm
import sys
import math
import numpy as np
import actionlib

class MoveBase:

    def __init__(self):
        rospy.init_node('odom_move_base')
        self.tw_pub = rospy.Publisher('base_controller/command', gm.Twist)
        #rospy.Subscriber('simple_move_base', gm.Pose2D, self.pose2d_cb)
        self.tl = tf.TransformListener()

        self.go_xy_server = actionlib.SimpleActionServer('go_xy', hm.GoXYAction, self._go_xy_cb)
        self.go_xy_server.start()

        self.go_ang_server = actionlib.SimpleActionServer('go_angle', hm.GoAngleAction, self._go_ang_cb)
        self.go_ang_server.start()

    def _go_xy_cb(self, goal):
        rospy.loginfo('received go_xy goal %f %f' % (goal.x, goal.y))
        def send_feed_back(error):
            feed_back_msg = hm.GoXYFeedback()
            feed_back_msg.error = np.linalg.norm(error)
            self.go_xy_server.publish_feedback(feed_back_msg)
        error = self.go_xy(np.matrix([goal.x, goal.y]).T, func=send_feed_back)
        result = hm.GoXYResult()
        result.error = np.linalg.norm(error)
        self.go_xy_server.set_succeeded(result)


    def _go_ang_cb(self, goal):
        rospy.loginfo('received go_angle goal %f' % (goal.angle))

        def send_feed_back(error):
            feed_back_msg = hm.GoAngleFeedback()
            feed_back_msg.error = error
            self.go_ang_server.publish_feedback(feed_back_msg)

        error = self.go_angle(goal.angle, func=send_feed_back)
        result = hm.GoAngleResult()
        result.error = error
        self.go_ang_server.set_succeeded(result)


    def go_xy(self, target_base, tolerance=.01, max_speed=.1, func=None):
        k = 2.
        self.tl.waitForTransform('base_footprint', 'odom_combined', rospy.Time(), rospy.Duration(10))
        rate = rospy.Rate(100)

        od_T_bf = tfu.transform('odom_combined', 'base_footprint', self.tl)
        target_odom = (od_T_bf * np.row_stack([target_base, np.matrix([0,1.]).T]))[0:2,0]
        #print 'target base', target_base.T
        #print 'target odom', target_odom.T

        while not rospy.is_shutdown():
            robot_odom = np.matrix(tfu.transform('odom_combined', 'base_footprint', self.tl)[0:2,3])
            #rospy.loginfo('odom %s' % str(robot_odom.T))

            diff_odom  = target_odom - robot_odom
            mag        = np.linalg.norm(diff_odom)
            #rospy.loginfo('error %s' % str(mag))
            if func != None:
                func(diff_odom)
            if mag < tolerance:
                break

            if self.go_xy_server.is_preempt_requested():
                self.go_xy_server.set_preempted()
                break

            direc    = diff_odom / mag
            speed = min(mag * k, max_speed)
            vel_odom = direc * speed
            #vel_odom = direc * mag * k
            #print mag*k, max_speed, speed

            bf_T_odom = tfu.transform('base_footprint', 'odom_combined', self.tl)
            vel_base = bf_T_odom * np.row_stack([vel_odom, np.matrix([0,0.]).T])
            #pdb.set_trace()
            #rospy.loginfo('vel_base %f %f' % (vel_base[0,0], vel_base[1,0]))

            tw = gm.Twist()
            tw.linear.x = vel_base[0,0]
            tw.linear.y = vel_base[1,0]
            #rospy.loginfo('commanded %s' % str(tw))
            self.tw_pub.publish(tw)
            rate.sleep()
        rospy.loginfo('finished go_xy %f' % np.linalg.norm(diff_odom))
        return diff_odom

    ##
    # delta angle
    def go_angle(self, target_odom, tolerance=math.radians(5.), max_ang_vel=math.radians(20.), func=None):
        self.tl.waitForTransform('base_footprint', 'odom_combined', rospy.Time(), rospy.Duration(10))
        rate = rospy.Rate(100)
        k = math.radians(80)

        #current_ang_odom = tr.euler_from_matrix(tfu.transform('base_footprint', 'odom_combined', self.tl)[0:3, 0:3], 'sxyz')[2]
        #target_odom = current_ang_odom + delta_ang

        while not rospy.is_shutdown():
            robot_odom = tfu.transform('base_footprint', 'odom_combined', self.tl)
            current_ang_odom = tr.euler_from_matrix(robot_odom[0:3, 0:3], 'sxyz')[2]
            diff = ut.standard_rad(current_ang_odom - target_odom)
            p = k * diff
            #print diff
            if func != None:
                func(diff)
            if np.abs(diff) < tolerance:
                break

            if self.go_ang_server.is_preempt_requested():
                self.go_ang_server.set_preempted()
                break

            tw = gm.Twist()
            vels = [p, np.sign(p) * max_ang_vel]
            tw.angular.z = vels[np.argmin(np.abs(vels))]
            #rospy.loginfo('diff %.3f vel %.3f' % (math.degrees(diff), math.degrees(tw.angular.z)))
            self.tw_pub.publish(tw)
            #rospy.loginfo('commanded %s' % str(tw))
            rate.sleep()
        rospy.loginfo('finished %.3f' % math.degrees(diff))
        return diff


if __name__ == '__main__':
    m = MoveBase()
    rospy.loginfo('simple move base server up!')
    rospy.spin()

