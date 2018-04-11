#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 11 16:57:54 2015

@author: cdondrup
"""

import rospy
from hrsi_representation.msg import QTCArray
from hrsi_state_prediction.msg import QTCPrediction, QTCPredictionArray
import json
import numpy as np
import hrsi_state_prediction.qtc_utils as qu
#import time
from qsrrep_ros.ros_client import ROSClient
from qsrrep_lib.rep_io_pf import PfRepRequestUpdate, PfRepRequestCreate, PfRepRequestRemove
from qsrrep_utils.qtc_model_creation import QTCModelCreation
from qsrrep_utils.hmm_model_creation import HMMModelCreation
from qsrrep_pf.pf_model import PfModel
from bayes_people_tracker.msg import PeopleTracker
from visualization_msgs.msg import MarkerArray
import people_tracker_emulator.msg_creator as mc
from std_msgs.msg import ColorRGBA
import os


class StatePredictor(object):
    __interaction_types = ["passby", "pathcrossing"]
    __filters = {}
    __classification_results = {}

    model = None
    rules = {}

    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self.client = ROSClient()
        qmc = QTCModelCreation()
        obs = qmc.create_observation_model(qtc_type=qmc.qtc_types.qtch, start_end=True)
        self.lookup = qmc.create_states(qtc_type=qmc.qtc_types.qtch, start_end=True)
        hmc = HMMModelCreation()
        m = PfModel()
        for f in os.listdir(rospy.get_param("~model_dir")):
            filename = rospy.get_param("~model_dir") + '/' + f
            if f.endswith(".hmm"):
                rospy.loginfo("Creating prediction model from: %s", filename)
                pred = hmc.create_prediction_model(input=filename)
                m.add_model(f.split('.')[0], pred, obs)
            elif f.endswith(".rules"):
                with open(filename) as fd:
                    rospy.loginfo("Reading rules from: %s", filename)
                    # Necessary due to old rule format:
                    self.rules[f.split('.')[0]] = self._create_proper_qtc_keys(json.load(fd))

        self.model = m.get()

        visualisation = rospy.get_param("~visualisation")
        self.visualisation_colors = visualisation['models']
        self.default_color = ColorRGBA(
            a=1.0,
            r=visualisation['default']['r']/255.,
            g=visualisation['default']['g']/255.,
            b=visualisation['default']['b']/255.
        )

        self.pub = rospy.Publisher("~prediction_array", QTCPredictionArray, queue_size=10)
        self.markpub = rospy.Publisher("~marker_array", MarkerArray, queue_size=10)
        rospy.Subscriber(rospy.get_param("~qtc_topic", "/online_qtc_creator/qtc_array"), QTCArray, self.callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("~ppl_topic", "/people_tracker/positions"), PeopleTracker, self.people_callback, queue_size=1)
        rospy.loginfo("... all done")

    def _create_proper_qtc_keys(self, dictionary):
        ret = {}
        for k,v in dictionary.items():
            if isinstance(v,dict):
                v = self._create_proper_qtc_keys(v)
            ret[','.join(k.replace('-1','-').replace('1','+').replace('9','').replace(',',''))] = v
        return ret

    def callback(self, msg):
        out = QTCPredictionArray()
        out.header = msg.header
        for q in msg.qtc:
            m = QTCPrediction()
            m.uuid = q.uuid
            if q.uuid not in self.__filters:
                self.client.call_service(
                    PfRepRequestCreate(
                        num_particles=1000,
                        models=self.model,
                        state_lookup_table=self.lookup,
                        uuid=q.uuid,
                        ensure_particle_per_state=True,
                        debug=True,
                        starvation_factor=0.1
                    )
                )
            self.__filters[q.uuid] = msg.header.stamp.to_sec()

            qtc_robot = json.loads(q.qtc_robot_human)[-1].split(',')
            qtc_goal = json.loads(q.qtc_goal_human)[-1].split(',')

            qtc = [qtc_goal[1], qtc_goal[3], qtc_robot[1]]
            if len(qtc_robot) == 4:
                qtc.append(qtc_robot[3])
            qtc = ','.join(qtc)
#            start = time.time()
            qtc_state = self.client.call_service(
                PfRepRequestUpdate(
                    uuid=q.uuid,
                    observation=qtc,
                    debug=True
                )
            )
#            print "+++ elapsed", time.time()-start
            if qtc_state == None:
                rospy.loginfo("%s state reported, aborting" % qtc_state)
                return
                
            rospy.loginfo("Current state according to filter: %s" % qtc_state)
            self.__classification_results[q.uuid] = qtc_state[2]
            try:
                states = self.rules[qtc_state[2]][qtc_state[0]].keys()
                probs = self.rules[qtc_state[2]][qtc_state[0]].values() # Both lists are always in a corresponding order
            except KeyError as e:
                rospy.logwarn("%s not in rules" % e)
                return
            pred = qtc_state[0].split(',')
            print pred
            prediction = states[probs.index(max(probs))].split(',')
            print prediction
            prediction = [prediction[0], pred[2]] \
                if len(pred) < 4 else [prediction[0], pred[2], prediction[1], pred[3]]
            prediction = ','.join(prediction)
            rospy.logdebug("Prediction: %s" %prediction)
            if prediction == None:
                return
            m.qtc_serialised = json.dumps(prediction)
            out.qtc.append(m)
        self.pub.publish(out)
        self.__decay(self.__filters, decay_time=10.)

    def _create_qtc_string(self, qtc):
        qtc = np.array(qtc)
        qtc = qtc[qtc!=9.]
        return ','.join(map(str, qtc.astype(int))).replace('-1','-').replace('1','+')

    def __decay(self, filters, decay_time=60.):
        for k in filters.keys():
            if filters[k] + decay_time < rospy.Time.now().to_sec():
                rospy.loginfo("Deleting particle filter: %s last seen %f" % (k, filters[k]))
                self.client.call_service(PfRepRequestRemove(uuid=k))
                del filters[k]

    def people_callback(self, msg):
        people = [[],[]]
        for uuid, pose in zip(msg.uuids, msg.poses):
            people[0].append(pose)
            try:
                people[1].append(
                    ColorRGBA(
                        a=1.0,
                        r=self.visualisation_colors[self.__classification_results[uuid]]['color']['r']/255.,
                        g=self.visualisation_colors[self.__classification_results[uuid]]['color']['g']/255.,
                        b=self.visualisation_colors[self.__classification_results[uuid]]['color']['b']/255.
                    )
                )
            except KeyError:
                people[1].append(self.default_color)

        self.markpub.publish(
            mc.marker_array_from_people_tracker_msg(
                poses=people[0],
                target_frame=msg.header.frame_id,
                color=people[1]
            )
        )


if __name__ == "__main__":
    rospy.init_node("qtc_state_predictor")
    s = StatePredictor(rospy.get_name())
    rospy.spin()
