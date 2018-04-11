#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 30 15:28:58 2015

@author: cdondrup
"""

import rospy
from hrsi_representation.file_input import FileInput
import hrsi_representation.output as output
from hrsi_representation.msg import QTCArray
import json


class OfflineQTCCreator(object):
    """trains hmm from raw data files"""

    def __init__(self, name):
        """Creates a new instance of the class
        """
        rospy.loginfo("Starting %s", name)

        self.input = rospy.get_param("~input")
        self.outpath = rospy.get_param("~outpath", "")
        self.qsr = rospy.get_param("~qsr", "qtccs")

        self.parameters = {"qtcs": {
            "quantisation_factor": rospy.get_param("~quantisation_factor", 0.01),
            "validate":            rospy.get_param("~validate", False),
            "no_collapse":         rospy.get_param("~no_collapse", True),
            "distance_threshold":  rospy.get_param("~distance_threshold", 2.)
        },
            "for_all_qsrs": {
                "qsrs_for": [("robot", "human"), ("goal", "human")]
        }}

        self.pub = rospy.Publisher("~qtc_array", QTCArray, queue_size=10)

        self.file_input = FileInput()

        if self.input == "":
            rospy.logfatal("No input path provided. Cannot load files.")
            return

    def create(self):
        rospy.loginfo("Reading from dir: '%s'" % self.input)
        data_rh, files = self.file_input.generate_data_from_input(
            path=self.input,
            k="agent1", k_x="x1", k_y="y1",
            l="agent2", l_x="x2", l_y="y2"
        )
        qtc_rh = self.file_input.convert(
            data=data_rh,
            qtc_type=self.qsr,
            parameters=self.parameters,
            argprobd=False
        )
        data_gh, files = self.file_input.generate_data_from_input(
            path=self.input,
            k="agent3", k_x="x3", k_y="y3",
            l="agent2", l_x="x2", l_y="y2"
        )
        qtc_gh = self.file_input.convert(
            data=data_gh,
            qtc_type="qtccs",
            parameters=self.parameters,
            argprobd=False
        )
        return qtc_rh, qtc_gh, files

    def publish(self, qtc_rh, qtc_gh, files):
        out = output.create_qtc_array_msg(
            stamp=rospy.Time.now()
        )
        for qrh, qgh, f in zip(qtc_rh, qtc_gh, files):
            m = output.create_qtc_msg(
                collapsed=not self.parameters["qtcs"]["no_collapse"],
                qtc_type=self.qsr,
                quantisation_factor=self.parameters["qtcs"]["quantisation_factor"],
                distance_threshold=self.parameters["qtcs"]["distance_threshold"] if isinstance(self.parameters["qtcs"]["distance_threshold"], float) else -1.0,
                abstract_distance_threshold=self.parameters["qtcs"]["distance_threshold"] if isinstance(self.parameters["qtcs"]["distance_threshold"], str) else '',
                smoothing_rate=0.0,
                validated=self.parameters["qtcs"]["validate"],
                uuid=f,
                qtc_robot_human=json.dumps(qrh[0].tolist()),
                prob_distance_robot_human=json.dumps(qrh[1][-len(qrh[0]):]), # Only add as amany distances as qtc states
                qtc_goal_human=json.dumps(qgh[0].tolist()),
                prob_distance_goal_human=json.dumps(qgh[1][-len(qgh[0]):]) # Only add as amany distances as qtc states

#                collapsed=not self.parameters["qtcs"]["no_collapse"],
#                qtc_type=self.qsr,
#                k=d["agent1"]["name"],
#                l=d["agent2"]["name"],
#                quantisation_factor=self.parameters["qtcs"]["quantisation_factor"],
#                distance_threshold=self.parameters["qtcs"]["distance_threshold"],
#                smoothing_rate=0.0,
#                validated=self.parameters["qtcs"]["validate"],
#                uuid=f,
#                qtc_serialised=json.dumps(q[0].tolist()),
#                prob_distance_serialised=""
            )
            out.qtc.append(m)

        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node("offline_qtc_creator")
    t = OfflineQTCCreator(rospy.get_name())
    qtc_rh, qtc_gh, files = t.create()
    t.publish(qtc_rh, qtc_gh, files)
    print len(qtc_rh[5][0]), len(qtc_gh[5][0])
    output.create_numpy_files(qtc_rh, qtc_gh, files, t.outpath)
#    if t.outpath != "":
#        output.write_files(qtc,files,t.outpath)
#    for elem, f in zip(qtc, files):
#        print f, elem
    #rospy.spin()


