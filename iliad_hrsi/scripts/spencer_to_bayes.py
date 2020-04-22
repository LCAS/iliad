#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""


@author: tejas

"""

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, Vector3
from bayes_people_tracker.msg import PeopleTracker
from spencer_tracking_msgs.msg import TrackedPersons
import numpy as np
import tf
import math


class SpencerToBayes:
    """Converts Spencer Tracked Person Input to Bayes People Tracker Output"""

    def __init__(self):
		person_topic         = rospy.get_param("~person_topic", "/robot4/perception/tracked_persons")
		ppl_topic            = rospy.get_param("~ppl_topic", "/robot4/people_tracker/positions")
		self.target_frame    = rospy.get_param("~target_frame", "robot4/base_link")
		self.listener        = tf.TransformListener()
		self.pub             = rospy.Publisher(ppl_topic, PeopleTracker, queue_size=1)
		self.last_msg        = PeopleTracker()
		rospy.Subscriber(
			person_topic,
			TrackedPersons,
			callback=self.person_callback
			#queue_size=10
		)

    def person_callback(self, msg):
		a = PeopleTracker()
		a.header = msg.header
		a.min_distance = np.inf
		a.min_distance_angle = 0
		for t in msg.tracks:
			b = np.uint8(t.track_id)
			a.uuids.append(str(b))
			
			person = Pose()

			person.position = t.pose.pose.position
			person.orientation = t.pose.pose.orientation
			a.poses.append(person)
			
			v = Vector3()
			v.x = t.twist.twist.linear.x
			v.y = t.twist.twist.linear.y
			v.z = t.twist.twist.linear.z
			a.velocities.append(v)
			
			person = PoseStamped()
			person.header = msg.header
			
			person.pose = t.pose.pose
			
			if msg.header.frame_id != self.target_frame:
				try:
					ti = self.listener.getLatestCommonTime(self.target_frame, msg.header.frame_id)
					msg.header.stamp = ti
					transformed_person = self.listener.transformPose(self.target_frame, person)
				except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
					rospy.logerr("["+rospy.get_name()+"]: " + "Can't transform, reason: "+str(ex))
					return None
			x1 = transformed_person.pose.position.x
			y1 = transformed_person.pose.position.y
			z1 = transformed_person.pose.position.z
			
			dis = round(math.sqrt(math.pow(x1,2)+math.pow(y1,2)+math.pow(z1,2)),3)

			a.distances.append(dis)

						
			#ang = math.atan2(math.sqrt(math.pow(x1,2)+math.pow(y1,2)),z1)
			ang = math.atan2(y1,x1)
			
			a.angles.append(ang)

			if dis < a.min_distance:
				a.min_distance = dis
				a.min_distance_angle = ang
		
		self.pub.publish(a)


if __name__ == "__main__":
    rospy.init_node("spencer_to_bayes")  # ,log_level=rospy.DEBUG)
    stb = SpencerToBayes()
    rospy.spin()
