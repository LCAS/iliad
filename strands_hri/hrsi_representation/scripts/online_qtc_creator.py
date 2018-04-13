#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 27 16:03:38 2015

@author: cdondrup
"""

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
from bayes_people_tracker.msg import PeopleTracker
from dynamic_reconfigure.server import Server as DynServer
from hrsi_representation.cfg import OnlineQTCCreatorConfig
from hrsi_representation.msg import QTCArray
import hrsi_representation.output as output
from hrsi_representation.online_input import OnlineInput
import numpy as np
import tf
import json
import thread
from multiprocessing.pool import ThreadPool
from collections import OrderedDict

class OnlineQTCCreator(object):
    """Creates QTC state sequences from online input"""

    # The order of this dict has to reflect the index of the qtc_type in the dynamic reconfigure file
    _qsr_relations_and_values = OrderedDict([
        ("int", (  (.46-.0)/2  +.0,    (.46-.0)/10)),
        ("per", ((1.22-.46)/2 +.46,  (1.22-.46)/10)),
        ("soc", ((3.7-1.22)/2+1.22,  (3.7-1.22)/10)),
        ("pub", ((6.0-3.7)/2  +3.7,   (6.0-3.7)/10)),
        ("und", ((10.0-6.0)/2 +6.0,  (10.0-6.0)/10))
    ])
    _robot_pose = None
    _goal_pose = PoseStamped(header=Header(frame_id="/map"))
    _buffer = dict()
    _smoothing_buffer = dict()
    _msg_buffer = []

    __thread_pool = ThreadPool(processes=2)

    def __init__(self, name):
        rospy.loginfo("Starting %s" % name)
        self.input           = OnlineInput()
        ppl_topic            = rospy.get_param("~ppl_topic", "/people_tracker/positions")
        robot_topic          = rospy.get_param("~robot_topic", "/robot_pose")
        robotSt_topic        = rospy.get_param("~robotST_topic", "/robotST_pose")

        goal_topic           = rospy.get_param("~goal_topic", "/move_base/current_goal")
        qtc_arr_topic        = rospy.get_param("~qtc_arr_topic", "~qtc_array")
        self.target_frame    = rospy.get_param("~target_frame", "/map")
        self.processing_rate = rospy.get_param("~processing_rate", 30)
        self.dyn_srv         = DynServer(OnlineQTCCreatorConfig, self.dyn_callback)
        self.listener        = tf.TransformListener()
        self.pub             = rospy.Publisher(qtc_arr_topic, QTCArray, queue_size=10)
        self.last_msg        = QTCArray()
        rospy.Subscriber(
            ppl_topic,
            PeopleTracker,
            callback=self.ppl_callback,
            queue_size=10
        )
        rospy.Subscriber(
            robot_topic,
            Pose,
            callback=self.pose_callback,
            queue_size=10
        )

        rospy.Subscriber(
            robotSt_topic,
            PoseStamped,
            callback=self.poseSt_callback,
            queue_size=10
        )


        rospy.Subscriber(
            goal_topic,
            PoseStamped,
            callback=self.goal_callback,
            queue_size=10
        )

        self.request_thread = thread.start_new(self.generate_qtc, ())

    def dyn_callback(self, config, level):
        self.decay_time = config["decay_time"]
        self.qtc_type = self.input.qtc_types[config["qtc_type"]]
        self.prune_buffer = config["prune_buffer"]
        self.max_buffer_size = config["max_buffer_size"]
        # If we prune the buffer, validate and no_callapse will have no effect.
        # Setting them to false to make that clear
        if self.prune_buffer:
            config["validate"]    = False
            config["no_collapse"] = False
        self.parameters = {
            self.qtc_type: {
                "quantisation_factor": config["quantisation_factor"],
                "distance_threshold":
                    config["distance_threshold"] if self.qtc_type != "qtcbcs_argprobd" \
                    else self._qsr_relations_and_values.keys()[config["abstract_distance_threshold"]],
                "validate": config["validate"],
                "no_collapse": config["no_collapse"]
            },
            "argprobd": {
                "qsr_relations_and_values": dict(self._qsr_relations_and_values)
            },
            "for_all_qsrs": {
                "qsrs_for": [("Robot", "Human"), ("Goal", "Human")]
            }
        }
        if self.qtc_type == "qtcbcs_argprobd":
            self.parameters[self.qtc_type]["qsr_relations_and_values"] = dict(self._qsr_relations_and_values)
        self.smoothing_rate = config["smoothing_rate"]
        return config

    def ppl_callback(self, msg):
        if self._robot_pose == None:
            return
        msgs = {
            "ppl": msg,
            "robot": self._robot_pose,
            "goal": self._goal_pose
        }
        self._msg_buffer.append(msgs)

    def poseSt_callback(self, msg):
        self.pose_callback(msg.pose)

    def pose_callback(self, msg):
        self._robot_pose = msg

    def goal_callback(self, msg):
        self._goal_pose = msg

    def _transform(self, msg, target_frame):
        if msg.header.frame_id != target_frame:
            try:
                t = self.listener.getLatestCommonTime(target_frame, msg.header.frame_id)
                msg.header.stamp = t
                return self.listener.transformPose(target_frame, msg)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                ##rospy.logdebug( "[" + rospy.get_name() + "]: " + "TF failed. Reason: " + str(ex))
                return None
        else:
            return msg

    def generate_qtc(self):
        rate = rospy.Rate(self.processing_rate)
        while not rospy.is_shutdown():
            if not self._msg_buffer:
                rate.sleep()
                continue

            # Get oldest buffer entry
            ppl_msg = self._msg_buffer[0]["ppl"]
            robot_msg = self._msg_buffer[0]["robot"]
            goal_msg = self._msg_buffer[0]["goal"]
            del self._msg_buffer[0]

            # Creating a new message
            out = output.create_qtc_array_msg(
                frame_id=self.target_frame
            )

            ##rospy.logdebug( "[" + rospy.get_name() + "]: " + "QTC created. ")
            # Looping through detected humans
            for (uuid, pose, angle) in zip(ppl_msg.uuids, ppl_msg.poses, ppl_msg.angles):
                # Transforming pose into target_frame if necessary
                ##rospy.logdebug( "[" + rospy.get_name() + "]: " + "Look! a person! ") 
                person = PoseStamped()
                person.header = ppl_msg.header
                person.pose = pose
                transformed_person = self._transform(person, self.target_frame)
#                transformed_goal = self._transform(goal_msg, self.target_frame)
                if transformed_person == None:
                    ##rospy.logdebug( "[" + rospy.get_name() + "]: " + "Couldn't transform... ")
                    continue

                if not uuid in self._smoothing_buffer.keys(): # No entry yet
                    ##rospy.logdebug( "[" + rospy.get_name() + "]: " + "Adding entry ")
                    self._smoothing_buffer[uuid] = {
                        "start_time": ppl_msg.header.stamp.to_sec(),
                        "data": np.array(
                            [
                                robot_msg.position.x,
                                robot_msg.position.y,
                                transformed_person.pose.position.x,
                                transformed_person.pose.position.y,
                                goal_msg.pose.position.x,
                                goal_msg.pose.position.y
                            ]
                    ).reshape(-1,6), "last_seen": ppl_msg.header.stamp}
                else: # Already in buffer
                    ##rospy.logdebug( "[" + rospy.get_name() + "]: " + "I have your entry ")
                    self._smoothing_buffer[uuid]["data"] = np.append(
                        self._smoothing_buffer[uuid]["data"],
                        [
                            robot_msg.position.x,
                            robot_msg.position.y,
                            transformed_person.pose.position.x,
                            transformed_person.pose.position.y,
                            goal_msg.pose.position.x,
                            goal_msg.pose.position.y
                        ]
                    ).reshape(-1,6)
                    self._smoothing_buffer[uuid]["last_seen"] = ppl_msg.header.stamp

            # Flush smoothing buffer and create QSR
            # Looping through smoothing buffer
            for uuid, data in self._smoothing_buffer.items():
                ##rospy.logdebug( "[" + rospy.get_name() + "]: " + "Smoothing buffer ... ")
                # If the smoothing time is not up, do nothing for this entry
                if not data["start_time"] + self.smoothing_rate <= ppl_msg.header.stamp.to_sec():
                    continue

                latest = np.array([ # Mean over the coordinates to smooth them
                    np.mean(data["data"][:,0]),
                    np.mean(data["data"][:,1]),
                    np.mean(data["data"][:,2]),
                    np.mean(data["data"][:,3]),
                    np.mean(data["data"][:,4]),
                    np.mean(data["data"][:,5])
                ])

                # Put smoothed values in buffer
                if not uuid in self._buffer.keys(): # No entry yet, create a new one
                    self._buffer[uuid] = {
                        "data": latest.reshape(-1,6),
                        "last_seen": data["last_seen"]
                    }
                else: # Already in buffer, append latest values
                    if not np.allclose(self._buffer[uuid]["data"][-1], latest):
                        self._buffer[uuid]["data"] = np.append(
                            self._buffer[uuid]["data"],
                            latest
                        ).reshape(-1,6)
                    # restrict buffer length to max_buffer_size
                    self._buffer[uuid]["data"] = self._buffer[uuid]["data"][-self.max_buffer_size:]
                self._buffer[uuid]["last_seen"] = data["last_seen"] # Add time of last update for decay

                del self._smoothing_buffer[uuid] # Delete element from smoothing buffer

                # If there are more than 1 entries in the buffer for this person
                # Create QTC representation
                if self._buffer[uuid]["data"].shape[0] > 1:
                    #rospy.logdebug( "[" + rospy.get_name() + "]: " + "More than 1 entry in the buffer for this one... ")
                    robot_thread = self.__thread_pool.apply_async(self.create_qsrs, (self._buffer[uuid]["data"][:,0:4], ("Robot", "Human"), self.qtc_type, self.parameters))
                    goal_thread = self.__thread_pool.apply_async(self.create_qsrs, (self._buffer[uuid]["data"][:,2:6], ("Human", "Goal"), "qtccs", self.parameters))
                    qsrs_robot = robot_thread.get()
                    qsrs_goal = goal_thread.get()

                    if self.prune_buffer:
                        self._buffer[uuid]["data"] = self._buffer[uuid]["data"][-1]
                    # Create new message
                    qtc_msg = output.create_qtc_msg(
                        collapsed=not self.parameters[self.qtc_type]["no_collapse"],
                        qtc_type=self.qtc_type,
                        quantisation_factor=self.parameters[self.qtc_type]["quantisation_factor"],
                        distance_threshold=self.parameters[self.qtc_type]["distance_threshold"] if isinstance(self.parameters[self.qtc_type]["distance_threshold"], float) else -1.0,
                        abstract_distance_threshold=self.parameters[self.qtc_type]["distance_threshold"] if isinstance(self.parameters[self.qtc_type]["distance_threshold"], str) else '',
                        smoothing_rate=self.smoothing_rate,
                        validated=self.parameters[self.qtc_type]["validate"],
                        uuid=uuid,
                        qtc_robot_human=json.dumps(qsrs_robot[0]),
                        prob_distance_robot_human=json.dumps(qsrs_robot[1][-len(qsrs_robot[0]):]), # Only add as many distances as qtc states
                        qtc_goal_human=json.dumps(qsrs_goal[0]),
                        prob_distance_goal_human=json.dumps(qsrs_goal[1][-len(qsrs_goal[0]):]) # Only add as many distances as qtc states
                    )

                    out.qtc.append(qtc_msg)
                    #rospy.logdebug( "[" + rospy.get_name() + "]: " + "out qtc vector has " + str(len(out.qtc)) + " entries")
                    out.header.stamp = self._buffer[uuid]["last_seen"]

            # If there is something to publish and it hasn't been published before, publish
            # If prune_buffer == True then we always publish
#            if out.qtc and (out.qtc != self.last_msg.qtc or self.prune_buffer):
            if out.qtc:
                self.pub.publish(out)
                #rospy.logerr( "[" + rospy.get_name() + "]: " + "PUBLISH.............................................! ")
#                self.last_msg = out
            self.decay(ppl_msg.header.stamp) # Delete old elements from buffer
            rate.sleep()

    def create_qsrs(self, coords, agents, qtc_type, parameters):
        #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Called! ")
        
        datum = self.input.generate_data_from_input(
                agent1=agents[0],
                agent2=agents[1],
                x1=coords[:,0],
                y1=coords[:,1],
                x2=coords[:,2],
                y2=coords[:,3]
            )
        
        #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Datum generated! ")
        ans =  self.input.convert(
            data=datum,
            qtc_type=qtc_type,
            parameters=parameters
        )[0]

        #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Processing finished")
        
        return ans

    def decay(self, last_time):
        for uuid in self._buffer.keys():
            if self._buffer[uuid]["last_seen"].to_sec() + self.decay_time < last_time.to_sec():
                rospy.loginfo("Deleted %s, last seen at %s" % (uuid, str(self._buffer[uuid]["last_seen"].to_sec())))
                del self._buffer[uuid]

if __name__ == "__main__":
    rospy.init_node("online_qtc_creator")#,log_level=rospy.DEBUG)
    oqc = OnlineQTCCreator(rospy.get_name())
    rospy.spin()
