#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
	person_topic         = rospy.get_param("~person_topic", "/robot1/spencer/perception/tracked_persons")
        ppl_topic            = rospy.get_param("~ppl_topic", "/robot1/people_tracker_filter/positions")
	self.target_frame    = rospy.get_param("~target_frame", "/robot1/base_link")
	self.listener        = tf.TransformListener()
	self.pub             = rospy.Publisher(ppl_topic, PeopleTracker)#, queue_size=10)
	self.last_msg        = PeopleTracker()
	rospy.Subscriber(
	    person_topic,
	    TrackedPersons,
	    callback=self.person_callback
	    #queue_size=10
	)

    def person_callback(self, msg):
	a = PeopleTracker()
	
	#print("msg tracks: ")
	#print(msg.tracks)
	#for t in msg.header:
	#	for s in a.header:
	#		s.seq = t.seq
	#		s.stamp = t.stamp
	#		s.frame_id = t.frame_id
	#		print("Frame ID: ")
	#		print(s.frame_id)
	a.header = msg.header
	
	print("header: ")
	print(a.header)
	for t in msg.tracks:
		b = np.uint8(t.track_id)
		a.uuids.append(str(b)) #= b#'track_%08d' %(t.track_id)
		
		print("track_id: ")
		print(t.track_id)
		print("uuids: ")
		print(a.uuids)
		print("poses: ")
		#print(t.pose)
		person = Pose()
		#person.header = a.header
		person.position = t.pose.pose.position
		person.orientation = t.pose.pose.orientation
		a.poses.append(person)# = t.pose
		
		print(a.poses)
		#for t1 in t.pose:
		#	for s in a.poses:
		#		for t2 in t1.pose:
		#			for t3 in t2.position:
		#				for s1 in s.position:
		#					s1.x = t3.x
		#					s1.y = t3.y
		#					s1.z = t3.z
		#			for t3 in t2.orientation:
		#				for s1 in s.orientation:
		#					s1.x = t3.x
		#					s1.y = t3.y
		#					s1.z = t3.z
		#					s1.w = t3.w
		v = Vector3()
		v.x = t.twist.twist.linear.x
		v.y = t.twist.twist.linear.y
		v.z = t.twist.twist.linear.z
		a.velocities.append(v)# = t.twist
		
		print("velocities: ")
		print(a.velocities)
		#for s in a.velocities:
		#	for t1 in t.twist:
		#		for t2 in t1.linear:
		#			s.x = t2.x
		#			s.y = t2.y
		#			s.z = t2.z
		person = PoseStamped()
		person.header = msg.header
                print(type(t.pose.pose))
		person.pose = t.pose.pose
		print("person pose: ")
                print(type(person))
		#print(person.pose)
		if msg.header.frame_id != self.target_frame:
			try:
				ti = self.listener.getLatestCommonTime(self.target_frame, msg.header.frame_id)
				msg.header.stamp = ti
				transformed_person = self.listener.transformPose(self.target_frame, person)
			except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
				return None
		x1 = transformed_person.pose.position.x
		y1 = transformed_person.pose.position.y
		z1 = transformed_person.pose.position.z
		print("x: ")
		print(x1)
		print("y: ")
		print(y1)
		print("z: ")
		print(z1)
		
		dis = [math.sqrt(math.pow(x1,2)+math.pow(y1,2)+math.pow(z1,2))]# = math.sqrt(math.pow(x,2)+math.pow(y,2)+math.pow(z,2))
		a.distances.append(dis[0])
		
		print("distances: ")
		print(a.distances)
		ang = [math.atan2(math.sqrt(math.pow(x1,2)+math.pow(y1,2)),z1)]# = math.atan2(math.sqrt(math.pow(x,2)+math.pow(y,2)),z)
		a.angles.append(ang[0])
		print("angles: ")
		print(a.angles)
		
		#self.pub.publish(a)
		#print("I just published")
		#break
		#a.distances.append(math.sqrt(math.pow(x,2)+math.pow(y,2)+math.pow(z,2)))
		#a.angles.append(math.atan2(math.sqrt(math.pow(x,2)+math.pow(y,2)),z))
	
	a.min_distance = min(a.distances)
	print("min distance: ")
	print(a.min_distance)
	i = 0
	for d in a.distances:
		if d == a.min_distance:
			break
		else:
			i = i + 1
	a.min_distance_angle = a.angles[i]
	print("min distance angle: ")
	print(a.min_distance_angle)
		
	print("I am publishing People Tracker object a.")
	
	self.pub.publish(a)
	#for s in a.distances:
	#	for s1 in a.poses:
	#		for s2 in s1.position:
	#			s.x = s2.x
	#			s.y = s2.y
	#			s.z = s2.z
	#for s in a.angles:
	#	for s1 in a.poses:
	#		for s2 in s1.orientation:
	#			s.x = s2.x
	#			s.y = s2.y
	#			s.z = s2.z
	#			s.w = s2.w
	#for s in a.distances:
	#	a.min_distance = min(s)
	#	for s1 in a.angles:
	#		if a.min_distance == s.x:
	#			a.min_distance_angle = s1.x
	#		elif a.min_distance == s.y:
	#			a.min_distance_angle = s1.y
	#		elif a.min_distance == s.z:
	#			a.min_distance_angle = s1.z
	#print("I am publishing People Tracker object a.")
	#self.pub.publish(a)
		#a.header.seq = t.seq
		#a.stamp = t.stamp
		#a.frame_id = t.frame_id
	#a.header = msg.header
	#for t in msg.tracks:
	#	a.uuids[] = msg.track_id
	#	a.poses[] = msg.pose.pose[]
	#return


if __name__ == "__main__":
    rospy.init_node("spencer_to_bayes")#,log_level=rospy.DEBUG)
    stb = SpencerToBayes()
    rospy.spin()
