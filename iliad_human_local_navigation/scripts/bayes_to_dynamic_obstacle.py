#!/usr/bin/env python

import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Point32, QuaternionStamped, Quaternion, TwistWithCovariance
from tf.transformations import quaternion_from_euler
from bayes_people_tracker.msg import PeopleTracker
import numpy as np

class BayesToDynamicObstacle:
	"""Converts Bayes People Tracker input to Dynamic Obstacle output"""
	
	def __init__(self):
		ppl_topic = rospy.get_param("~ppl_topic", "/robot4/people_tracker/positions")
		obstacle_msg = rospy.get_param("~obstacle_msg", "/move_base/TebLocalPlannerROS/obstacles")
		self.pub = rospy.Publisher(obstacle_msg,ObstacleArrayMsg,queue_size=1)
		rospy.Subscriber(ppl_topic,PeopleTracker,callback=self.people_tracker_callback)

	def people_tracker_callback(self, msg):
		obstacle_msg = ObstacleArrayMsg()
		#obstacle_msg.header = msg.header
		obstacle_msg.header.stamp = rospy.Time.now()
		obstacle_msg.header.frame_id = "world"
		
		i = 0

		for j in msg.uuids:
			obstacle_msg.obstacles.append(ObstacleMsg())
			obstacle_msg.obstacles[i].id = np.int64(msg.uuids[i])
			obstacle_msg.obstacles[i].polygon.points = [Point32()]
			obstacle_msg.obstacles[i].polygon.points[0].x = msg.poses[i].position.x
			obstacle_msg.obstacles[i].polygon.points[0].y = msg.poses[i].position.y
			obstacle_msg.obstacles[i].polygon.points[0].z = msg.poses[i].position.z
			obstacle_msg.obstacles[i].orientation.x = msg.poses[i].orientation.x
			obstacle_msg.obstacles[i].orientation.y = msg.poses[i].orientation.y
			obstacle_msg.obstacles[i].orientation.z = msg.poses[i].orientation.z
			obstacle_msg.obstacles[i].orientation.w = msg.poses[i].orientation.w
			
			obstacle_msg.obstacles[i].velocities.twist.linear.x = msg.velocities[i].x
			obstacle_msg.obstacles[i].velocities.twist.linear.y = msg.velocities[i].y
			obstacle_msg.obstacles[i].velocities.twist.linear.z = msg.velocities[i].z
			obstacle_msg.obstacles[i].velocities.twist.angular.x = 0
			obstacle_msg.obstacles[i].velocities.twist.angular.y = 0
			obstacle_msg.obstacles[i].velocities.twist.angular.z = 0
			
			i = i + 1

		print("I am publishing Dynamic Obstacles message obstacle_msg.")
		self.pub.publish(obstacle_msg)

if __name__ == '__main__':
	rospy.init_node("bayes_to_dynamic_obstacle")
	btdo = BayesToDynamicObstacle()
	rospy.spin()
