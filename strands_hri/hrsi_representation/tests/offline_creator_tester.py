#!/usr/bin/env python
PKG = 'hrsi_representation'
NAME = 'offline_creator_tester'

import rospy
import sys
import unittest
from hrsi_representation.msg import QTCArray
import subprocess
import json
import rospkg
import numpy as np
from roslib.packages import find_resource


class TestOfflineQTC(unittest.TestCase):
    TEST_DIR = rospkg.RosPack().get_path(PKG)+'/tests/data/'
    EXECUTABLE = find_resource(PKG, 'offline_qtc_creator.py')[0]

    _correct = {
        "qtcbs": [[-1, -1], [0, -1], [0, 0], [1, 0], [1, 1]],
        "qtccs": [[-1, -1, 0, 0], [-1, -1, 0, 1], [0, -1, 0, 1], [0, -1, 1, 1], [0, 0, 1, 1], [0, 0, 0, 1], [1, 0, 0, 1], [1, 1, 0, 1], [1, 1, 0, 0]],
        "qtcbcs": [[-1.0, -1.0, np.NaN, np.NaN], [0.0, -1.0, np.NaN, np.NaN], [0.0, -1.0, 0.0, 1.0], [0.0, -1.0, 1.0, 1.0], [0.0, 0.0, 1.0, 1.0], [0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 1.0], [1.0, 1.0, 0.0, 1.0], [1.0, 1.0, np.NaN, np.NaN]]
    }

    def __init__(self, *args):
        super(TestOfflineQTC, self).__init__(*args)

        rospy.init_node(NAME)
        self.test = []

    def _callback(self, msg):
        self.seq = msg.header.seq
        if msg.qtc:
            self.test=json.loads(msg.qtc[0].qtc_serialised)

    def test_offline_creator_qtcb(self):
        rospy.Subscriber("/offline_qtc_creator/qtc_array", QTCArray, callback=self._callback)
        p = subprocess.Popen(self.EXECUTABLE+" _input:="+self.TEST_DIR+" _qsr:=qtcbs", stdin=subprocess.PIPE, shell=True)
        while p.poll() == None:
            rospy.sleep(1)
#        print self.test
        self.assertEqual(self.test, self._correct["qtcbs"])

    def test_offline_creator_qtcc(self):
        rospy.Subscriber("/offline_qtc_creator/qtc_array", QTCArray, callback=self._callback)
        p = subprocess.Popen(self.EXECUTABLE+" _input:="+self.TEST_DIR+" _qsr:=qtccs", stdin=subprocess.PIPE, shell=True)
        while p.poll() == None:
            rospy.sleep(1)
#        print self.test
        self.assertEqual(self.test, self._correct["qtccs"])

    def test_offline_creator_qtcbc(self):
        rospy.Subscriber("/offline_qtc_creator/qtc_array", QTCArray, callback=self._callback)
        p = subprocess.Popen(self.EXECUTABLE+" _input:="+self.TEST_DIR+" _qsr:=qtcbcs", stdin=subprocess.PIPE, shell=True)
        while p.poll() == None:
            rospy.sleep(1)
#        print self.test
        np.testing.assert_equal(self.test, self._correct["qtcbcs"])


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestOfflineQTC, sys.argv)
#    p = TestOfflineQTC(sys.argv)
#    p.test_offline_creator_qtcb()
#    p.test_offline_creator_qtcc()
#    p.test_offline_creator_qtcbc()
