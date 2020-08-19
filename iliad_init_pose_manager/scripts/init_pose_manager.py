#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
Initial Pose manager

# 1.- check if we have saved pose (./ros/initialPose) and publish it into (intialPoseTopic)
# 2.- subscribe to robot pose at (RobotPoseTopic)
# 3.- save it on shutdown

"""

import os
import rospy
from rospy_message_converter import json_message_converter
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class InitialPoseManager(object):
    def __init__(self, name):
        rospy.loginfo("["+rospy.get_name()+"] " + "Starting ... ")

        # ................................................................
        # read ros parameters
        self.loadROSParams()
 
        # ................................................................
        # other params


        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()
        rospy.loginfo("["+rospy.get_name()+"] " + "Init done.")

        self.check_init_pose()
        r = rospy.Rate(self.robot_pose_update_rate)
        while not rospy.is_shutdown():
            msg=rospy.wait_for_message(self.robot_pose_topic_name, Odometry)
            self.robot_pose_callback(msg)
            r.sleep() 
        


    
    def check_init_pose(self):
        # ................................................................
        # read saved inital pose and publish it ...
        try:
            with open(self.saved_pose_uri, 'r') as infile:
                pose_str = infile.read()
            oldPose = json_message_converter.convert_json_to_ros_message('geometry_msgs/PoseWithCovarianceStamped', pose_str)
            
            initPose = PoseWithCovarianceStamped()
            initPose.pose = oldPose.pose
            initPose.header.frame_id = oldPose.header.frame_id
            initPose.header.stamp = rospy.Time.now()
            rospy.loginfo("["+rospy.get_name()+"] " + "Initial pose is:\n" + str(initPose))

            rospy.sleep(1)
            while self.initial_pose_topic_pub.get_num_connections() == 0:
                rospy.logwarn("["+rospy.get_name()+"] " + "Waiting for subscribers to connect to initpose")
                rospy.sleep(1)

            self.initial_pose_topic_pub.publish(initPose)
            rospy.loginfo("["+rospy.get_name()+"] " + "Initial pose found and published.")
        except IOError as e:
            rospy.logerr("["+rospy.get_name()+"] " + ": " + str(e) )


    def initROS(self):
        # publishers
        self.initial_pose_topic_pub = rospy.Publisher(self.initial_pose_topic_name, PoseWithCovarianceStamped, queue_size=1)

        # service clients        
        # ... none here

        # subscribers and listeners
        # we don't need to update position at the same rate it's published
        #rospy.Subscriber(self.robot_pose_topic_name, Odometry, self.robot_pose_callback, queue_size=1)                

        # service servers
        # ... none here
        # Timers
        # ... none here

        # Others
        rospy.on_shutdown(self.save_pose)
    # .............................................................................................................

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 5)

        # Initial Pose Topic topic
        self.initial_pose_topic_name = rospy.get_param(
            '~initial_pose_topic_name', '/robot'+str(self.robot_id)+'/initialpose')

        # Robot (corrected) Odometry topic 
        self.robot_pose_topic_name = rospy.get_param(
            '~robot_pose_topic_name', '/robot' + str(self.robot_id) + '/mcl_pose_estimate')

        # Robot (corrected) Odometry topic 
        self.robot_pose_update_rate = rospy.get_param(
            '~robot_pose_update_rate', 2)


        # File URI containing saved poses
        self.saved_pose_uri = rospy.get_param(
            '~saved_pose_uri', os.getenv("HOME") + '/.ros/lastKnownPose.json')
      
    # we subscribe to a pose publisher to get robot position
    def robot_pose_callback(self, msg):    
            self.robotPoseSt = PoseWithCovarianceStamped()
            self.robotPoseSt.header.frame_id = msg.header.frame_id
            self.robotPoseSt.pose = msg.pose            

    def save_pose(self):
        try:
            rospy.loginfo("["+rospy.get_name()+"] " + "... saving current pose:\n" + str(self.robotPoseSt))
            jsonPose =json_message_converter.convert_ros_message_to_json(self.robotPoseSt)
            with open(self.saved_pose_uri, 'w+') as outfile:
                 outfile.write(jsonPose)
        except AttributeError:
            rospy.logwarn("["+rospy.get_name()+"] " + "No saved pose to store.")



if __name__ == "__main__":
    rospy.init_node("Initial_Pose_Manager") #, log_level=rospy.DEBUG)
     # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        v = InitialPoseManager(rospy.get_name())
    except rospy.ROSInterruptException:
        pass



