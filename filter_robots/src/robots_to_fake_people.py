#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from visualization_msgs.msg import MarkerArray, Marker
from orunav_msgs.msg import RobotReport

class robots_to_fake_people():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # read parameters
        self.links_to_add = rospy.get_param('~links_to_add',"/base_link,/laser2d_top_link,/laser2d_floor_link")
        self.frame_id = rospy.get_param('~frame_id',"map_laser2d")
        self.publishing_rate = rospy.get_param('~publishing_rate',50) # Hz
        self.publish_markers = rospy.get_param('~publish_markers',True)
        self.radius = rospy.get_param('~marker_radius',0.5)

        # init variables
        self.links_to_add = self.links_to_add.split(",")
        self.active_robots = [0,0,0,0,0,0,0,0,0]

        # subscribers
        rospy.Subscriber("/robot1/control/report", RobotReport, self.robot1_status_callback,queue_size=1)
        rospy.Subscriber("/robot2/control/report", RobotReport, self.robot2_status_callback,queue_size=1)
        rospy.Subscriber("/robot3/control/report", RobotReport, self.robot3_status_callback,queue_size=1)
        rospy.Subscriber("/robot4/control/report", RobotReport, self.robot4_status_callback,queue_size=1)
        rospy.Subscriber("/robot5/control/report", RobotReport, self.robot5_status_callback,queue_size=1)
        rospy.Subscriber("/robot6/control/report", RobotReport, self.robot6_status_callback,queue_size=1)
        rospy.Subscriber("/robot7/control/report", RobotReport, self.robot7_status_callback,queue_size=1)
        rospy.Subscriber("/robot8/control/report", RobotReport, self.robot8_status_callback,queue_size=1)
        rospy.Subscriber("/robot9/control/report", RobotReport, self.robot9_status_callback,queue_size=1)
        
        self.tf_listener = tf.TransformListener()

        # publishers
        self.robot_fake_people_pub =rospy.Publisher("/robots_fake_people",TrackedPersons,queue_size=1)
        self.robot_circles_pub = rospy.Publisher('/robot_circles', MarkerArray,queue_size=10)  

        # services

        # timers

        self.run()


    def run(self):
        r = rospy.Rate(self.publishing_rate) # Hz 
        while not rospy.is_shutdown():
            #publish fake people associated to robot
            self.create_fake_people()
            r.sleep()


    def create_fake_people(self):
        TrackedPersons_msg = TrackedPersons()
        TrackedPersons_msg.header.stamp = rospy.get_rostime()
        TrackedPersons_msg.header.frame_id = self.frame_id

        for r in range(0, len(self.active_robots)):
            if self.active_robots[r] == 1: # if the robot has been report as active check the links specified
                for link in self.links_to_add:
                    try:
                        (trans,rot) = self.tf_listener.lookupTransform(self.frame_id,"/robot"+str(r+1)+link, rospy.Time(0))
                        TrackedPerson_msg = TrackedPerson()
                        TrackedPerson_msg.pose.pose.position.x = trans[0]
                        TrackedPerson_msg.pose.pose.position.y = trans[1]
                        TrackedPersons_msg.tracks.append(TrackedPerson_msg)

                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue                    

        self.robot_fake_people_pub.publish(TrackedPersons_msg)
        
        if self.publish_markers:
            self.create_circle_markers(TrackedPersons_msg)        


    def create_circle_markers(self,tracked_persons_msg):
        circle_marker_array_msg = MarkerArray()
        for c in range(0, len(tracked_persons_msg.tracks)):
            circle_marker_msg = Marker()
            circle_marker_msg.header.frame_id = self.frame_id
            circle_marker_msg.header.stamp = rospy.get_rostime()
            circle_marker_msg.type = 3 #cylinder
            circle_marker_msg.id = c
            circle_marker_msg.action = 0 # add
            circle_marker_msg.scale.x = self.radius * 2
            circle_marker_msg.scale.y = self.radius * 2
            circle_marker_msg.scale.z = self.radius * 2
            circle_marker_msg.color.b = 1
            circle_marker_msg.color.a = 0.5
            circle_marker_msg.pose.position.x = tracked_persons_msg.tracks[c].pose.pose.position.x
            circle_marker_msg.pose.position.y = tracked_persons_msg.tracks[c].pose.pose.position.y
            circle_marker_msg.pose.position.z = self.radius
            circle_marker_array_msg.markers.append(circle_marker_msg)

        self.robot_circles_pub.publish(circle_marker_array_msg)

    def robot1_status_callback(self,msg):
        self.active_robots[0] = 1

    def robot2_status_callback(self,msg):
        self.active_robots[1] = 1

    def robot3_status_callback(self,msg):
        self.active_robots[2] = 1

    def robot4_status_callback(self,msg):
        self.active_robots[3] = 1

    def robot5_status_callback(self,msg):
        self.active_robots[4] = 1

    def robot6_status_callback(self,msg):
        self.active_robots[5] = 1

    def robot7_status_callback(self,msg):
        self.active_robots[6] = 1

    def robot8_status_callback(self,msg):
        self.active_robots[7] = 1

    def robot9_status_callback(self,msg):
        self.active_robots[8] = 1


# Main function.
if __name__ == '__main__':
    rospy.init_node('robots_to_fake_people_node', log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        r2fp = robots_to_fake_people()
    except rospy.ROSInterruptException:
        pass
