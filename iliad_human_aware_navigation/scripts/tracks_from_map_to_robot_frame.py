#!/usr/bin/env python
import rospy, math
import tf
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson, TrackedPersons, TrackedPerson
from geometry_msgs.msg import Quaternion, PoseStamped, Vector3Stamped
from gazebo_msgs.msg import ModelStates
from collections import deque
import numpy as np
from std_msgs.msg import Float64

def tracked_persons_callback(trackedPersons_map_frame):
    trackedPersons_robot_frame = TrackedPersons()
    trackedPersons_robot_frame.header.stamp.secs = trackedPersons_map_frame.header.stamp.secs-0.03
    trackedPersons_robot_frame.header.stamp.nsecs = trackedPersons_map_frame.header.stamp.nsecs
    trackedPersons_robot_frame.header.frame_id = "robot1/base_link"

    exception = False
    for track in trackedPersons_map_frame.tracks:
        trackedPerson_robot_frame = TrackedPerson()
        trackedPerson_robot_frame.track_id = track.track_id
        trackedPerson_robot_frame.is_occluded = track.is_occluded
        trackedPerson_robot_frame.is_matched = track.is_matched
        trackedPerson_robot_frame.detection_id = track.detection_id
        trackedPerson_robot_frame.age = track.age

        # try:
        #transform pose
        tmp_pose_msg = PoseStamped()
        tmp_pose_msg.header.stamp = trackedPersons_robot_frame.header.stamp#trackedPersons_map_frame.header.stamp
        tmp_pose_msg.header.frame_id = "world"
        tmp_pose_msg.pose = track.pose.pose
        tmp_pose_msg_robot_frame = tf_listener.transformPose("robot1/base_link",tmp_pose_msg)  
        trackedPerson_robot_frame.pose.pose = tmp_pose_msg_robot_frame.pose

        #transform twist
        tmp_vector3_msg = Vector3Stamped()
        tmp_vector3_msg.header.stamp = trackedPersons_robot_frame.header.stamp#trackedPersons_map_frame.header.stamp
        tmp_vector3_msg.header.frame_id = "world"
        tmp_vector3_msg.vector = track.twist.twist.linear
        tmp_vector3_msg_linear = tf_listener.transformVector3("robot1/base_link",tmp_vector3_msg)
        trackedPerson_robot_frame.twist.twist.linear = tmp_vector3_msg_linear.vector

        # tmp_vector3_msg = Vector3Stamped()
        # tmp_vector3_msg.header.stamp = trackedPersons_map_frame.header.stamp
        # tmp_vector3_msg.header.frame_id = "world"
        # tmp_vector3_msg.vector = track.twist.twist.angular
        # tmp_vector3_msg_angular = tf_listener.transformVector3("robot1/base_link",tmp_vector3_msg)
        # trackedPerson_robot_frame.twist.twist.angular = tmp_vector3_msg_angular.vector

        trackedPersons_robot_frame.tracks.append(trackedPerson_robot_frame) 

        # except:
        #     exception = True

    if exception==False:
        track_robot_frame_publisher.publish(trackedPersons_robot_frame)

# Initialize node
rospy.init_node("tracks_from_map_to_robot_frame_node") #, log_level=rospy.DEBUG)

tf_listener = tf.TransformListener()
tf_listener.waitForTransform("robot1/base_link","world",rospy.Time(),rospy.Duration(20))

track_robot_frame_publisher = rospy.Publisher("robot1/human_perception/tracked_persons_in_base_link", TrackedPersons, queue_size=1)
track_map_frame_subscriber = rospy.Subscriber("robot1/human_perception/tracked_persons_in_map_frame", TrackedPersons, tracked_persons_callback, queue_size=1)

rospy.loginfo("Re-publishing persons tracks in robot frame")
rospy.spin()