#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
#import thread


class GoalPoseRepublisher(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
#        self.pose = PoseStamped()
        self.pub = rospy.Publisher(rospy.get_param("~out_topic", "~pose"), PoseStamped, queue_size=1, latch=True)
#        self.t = thread.start_new(self.publish, ())
        rospy.Subscriber(rospy.get_param("~in_topic", "/move_base/current_goal"), PoseStamped, self.callback)
        rospy.loginfo("... all done %s" %name)

    def callback(self, msg):
        self.pub.publish(msg)
#        self.pose = msg

#    def publish(self):
#        while not rospy.is_shutdown():
#            self.pub.publish(self.pose)
#            rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("goal_pose_republisher")
    g = GoalPoseRepublisher(rospy.get_name())
    rospy.spin()
