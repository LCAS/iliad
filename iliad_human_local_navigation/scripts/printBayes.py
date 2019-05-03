#!/usr/bin/env python

'''
Subscribes to a people_tracker and publishes in rviz markers

'''

import tf
from geometry_msgs.msg import PoseStamped
from bayes_people_tracker.msg import PeopleTracker
import rospy
import visualization_msgs
import geometry_msgs
import std_msgs
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
import colorsys
import math

# used for visualization


'''
Methods based on codes/human_perception/bayes_people_tracker/include/people_tracker/people_tracker.h
'''


class humanPrinter():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()
        # ................................................................
        # other params
        self.marker_seq = 0
        self.marker_ns = 'payos'
        self.markersArray = MarkerArray()

        # ................................................................
        rospy.loginfo("Node '%s' started.", rospy.get_name())

        rospy.spin()

    def loadROSParams(self):
        self.markersTopic = rospy.get_param('~markers_topic', "humanMarker")
        self.trackedHumansTopic = rospy.get_param(
            '~tracked_humans_topic', '/robot5/people_tracker/positions')

    def initROS(self):
        self.markersPub = rospy.Publisher(self.markersTopic, MarkerArray, queue_size=1)
        rospy.Subscriber(self.trackedHumansTopic, PeopleTracker,
                         callback=self.people_tracker_callback)

    def people_tracker_callback(self, msg):
        # rospy.loginfo("[%s]: Processing bag: " + self.bagFileName, rospy.get_name())
        id = 0
        self.markersArray = MarkerArray()
        self.target_frame = msg.header.frame_id

        for i in range(len(msg.poses)):
            human = self.createHuman(id, msg.poses[i])
            id = id + 6
            self.marker_ns = msg.uuids[i]
            # msg.velocities[i]
            self.markersArray.markers.extend(human)

        self.markersPub.publish(self.markersArray)

    def createMarker(self, id, type, action, pose, scale, color):
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = rospy.get_rostime()
        self.marker_seq = self.marker_seq + 1
        marker.header.seq = self.marker_seq
        marker.ns = self.marker_ns
        marker.id = id
        marker.type = type
        marker.action = action
        marker.pose = pose
        marker.scale = scale
        marker.color = color
        marker.lifetime = rospy.Duration(0)
        return marker

    def createHead(self, id, action, pose):
        scale = geometry_msgs.msg.Vector3()
        scale.x = 0.4
        scale.y = 0.4
        scale.z = 0.4
        color = std_msgs.msg.ColorRGBA()
        color.a = 1.0
        color.r = 233.0/255.0
        color.g = 150.0/255.0
        color.b = 122.0/255.0
        pose.position.z = 1.9
        return self.createMarker(id, visualization_msgs.msg.Marker.SPHERE, action, pose, scale, color)

    def createBody(self, id, action, pose):
        scale = geometry_msgs.msg.Vector3()
        scale.x = 0.35
        scale.y = 0.35
        scale.z = 2.7
        color = std_msgs.msg.ColorRGBA()
        color.a = 1.0
        color.r = 139.0/255.0
        color.g = 0.0/255.0
        color.b = 0.0/255.0
        pose.position.z = 1.1
        return self.createMarker(id, visualization_msgs.msg.Marker.CYLINDER, action, pose, scale, color)

    def createLegs(self, idl, idr, action, pose):
        legs = []
        scale = geometry_msgs.msg.Vector3()
        scale.x = 0.15
        scale.y = 0.2
        scale.z = 0.8
        color = std_msgs.msg.ColorRGBA()
        color.a = 1.0
        color.r = 0.0/255.0
        color.g = 0.0/255.0
        color.b = 139.0/255.0
        legs.append(self.createMarker(idl, visualization_msgs.msg.Marker.CYLINDER, action,
                                      self.generate_extremity_position(pose, 0.3, 0.0, 0.4), scale, color))
        legs.append(self.createMarker(idr, visualization_msgs.msg.Marker.CYLINDER, action,
                                      self.generate_extremity_position(pose, -0.3, 0.0, 0.4), scale, color))
        return legs

    def createArms(self, idl, idr, action, pose):
        arms = []
        scale = geometry_msgs.msg.Vector3()
        scale.x = 0.1
        scale.y = 0.1
        scale.z = 0.7
        color = std_msgs.msg.ColorRGBA()
        color.a = 1.0
        color.r = 139.0/255.0
        color.g = 0.0/255.0
        color.b = 0.0/255.0
        arms.append(self.createMarker(idl, visualization_msgs.msg.Marker.CYLINDER, action,
                                      self.generate_extremity_position(pose, 0.5, 0.0, 1.1), scale, color))
        arms.append(self.createMarker(idr, visualization_msgs.msg.Marker.CYLINDER, action,
                                      self.generate_extremity_position(pose, -0.5, 0.0, 1.1), scale, color))
        return arms

    def createHuman(self, id, pose):
        human = []
        #human.append(self.createHead(id, visualization_msgs.msg.Marker.ADD, pose))
        #id = id + 1
        human.append(self.createBody(id, visualization_msgs.msg.Marker.ADD, pose))
        id = id + 1
        #legs = self.createLegs(id, id+1, visualization_msgs.msg.Marker.ADD, pose)
        # human.extend(legs)
        #id = id + 2
        #arms = self.createArms(id, id+1, visualization_msgs.msg.Marker.ADD, pose)
        # human.extend(arms)
        # id = id + 2
        return human

    def generate_extremity_position(self, centre, dx, dy, z):

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [centre.orientation.x, centre.orientation.y, centre.orientation.z, centre.orientation.w])
        angle = yaw + math.pi/2.0
        p = centre.position
        p.z = z
        centre.position = self.generate_position(p, angle, dx, dy)
        return centre

    def generate_position(self, centre, angle, dx, dy):
        s = math.sin(angle)
        c = math.cos(angle)

        # rotate point
        res = geometry_msgs.msg.Point()
        res.x = dx * c - dy * s
        res.y = dx * s + dy * c

        # translate point back:
        res.x = res.x + centre.x
        res.y = res.y + centre.y
        res.z = centre.z
        return res


# Main function.
if __name__ == '__main__':
    rospy.init_node('humanPrint_node')  # , log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = humanPrinter()
    except rospy.ROSInterruptException:
        pass
