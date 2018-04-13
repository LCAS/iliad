#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Created on Mon Jul 27 16:48:09 2015

@author: cdondrup
"""

import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from hrsi_state_prediction.msg import QTCPredictionArray
from bayes_people_tracker.msg import PeopleTracker
import json
from hrsi_velocity_costmaps.costmap_creator import CostmapCreator
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from dynamic_reconfigure.server import Server as DynServer
from hrsi_velocity_costmaps.cfg import VelocityCostmapsConfig
from visualization_msgs.msg import Marker
from tf import TransformListener


class VelocityCostmapServer(object):
    def __init__(self, name):
        rospy.loginfo("["+rospy.get_name()+"] " + "Starting ... ")

        self.vis_marker_topic = rospy.get_param("~vis_marker_topic","~visualization_marker")
        self.vel_costmap_topic = rospy.get_param("~vel_costmap_topic","/velocity_costmap")
        self.origin_topic = rospy.get_param("~origin_topic","~origin")
        self.base_link_tf = rospy.get_param("~base_link_tf","base_link")

        self.vis_pub = rospy.Publisher(self.vis_marker_topic, Marker, queue_size=1)
        self.tf = TransformListener()
        self.cc = CostmapCreator(
            rospy.Publisher(self.vel_costmap_topic, OccupancyGrid, queue_size=10, latch=True),
            rospy.Publisher(self.origin_topic, PoseStamped, queue_size=10)
        )
        self.dyn_srv = DynServer(VelocityCostmapsConfig, self.dyn_callback)
        subs = [
            Subscriber(rospy.get_param("~qtc_topic", "/qtc_state_predictor/prediction_array"), QTCPredictionArray),
            Subscriber(rospy.get_param("~ppl_topic", "/people_tracker/positions"), PeopleTracker)
        ]
        self.ts = ApproximateTimeSynchronizer(
            fs=subs,
            queue_size=60,
            slop=0.1
        )
        self.ts.registerCallback(self.callback)
        rospy.loginfo("["+rospy.get_name()+"] " + "... all done.")

    def dyn_callback(self, config, level):
        self.cc.resolution = config["costmap_resolution"]
        self.cc.min_costs  = config["min_costs"]
        self.cc.max_costs  = config["max_costs"]
        return config

    def callback(self, qtc, ppl):
        vels = []
        #rospy.logerr("["+rospy.get_name()+"] " + "data received!.............................. ")
        try:
            t = self.tf.getLatestCommonTime(self.base_link_tf, ppl.header.frame_id)
            vs = Vector3Stamped(header=ppl.header)
            vs.header.stamp = t
            for v in ppl.velocities:
                vs.vector = v
                vels.append(self.tf.transformVector3(self.base_link_tf, vs).vector)
        except Exception as e:
            rospy.logerr("["+rospy.get_name()+"] " + str(e))
            return
        data_buffer = {
            e.uuid: {
                "qtc": json.loads(e.qtc_serialised),
                "angle": ppl.angles[ppl.uuids.index(e.uuid)],
                "velocity": vels[ppl.uuids.index(e.uuid)]
            } for e in qtc.qtc
        }
        try:
            element = data_buffer[ppl.uuids[ppl.distances.index(ppl.min_distance)]] # Only taking the closest person for now
            self.publish_closest_person_marker(ppl.poses[ppl.distances.index(ppl.min_distance)], ppl.header.frame_id)
        except KeyError as e:
            rospy.logerr("["+rospy.get_name()+"] " + str(e))
            return
        qtc = element["qtc"].split(',')
        self.cc.publish(
            angle=element["angle"],
            qtc_symbol=','.join([qtc[0]] if len(qtc) == 2 else [qtc[0], qtc[2]]),
            velocity=element["velocity"]
        )
        #rospy.logerr("["+rospy.get_name()+"] " + "costmap published!.............................. ")

    def publish_closest_person_marker(self, pose, frame_id):
        if self.vis_pub.get_num_connections() > 0:
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = frame_id
            m.ns = "velocity_costmaps"
            m.id = 0
            m.type = m.SPHERE
            m.action = m.MODIFY
            m.pose = pose
            m.pose.position.z = 2.0
            m.scale.x = .25
            m.scale.y = .25
            m.scale.z = .25
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.lifetime = rospy.Duration(1.)
            self.vis_pub.publish(m)

if __name__ == "__main__":
    rospy.init_node("velocity_costmap_server")
    v = VelocityCostmapServer(rospy.get_name())
    rospy.spin()
