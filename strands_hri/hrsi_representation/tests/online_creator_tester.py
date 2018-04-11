#!/usr/bin/env python
PKG = 'hrsi_representation'
NAME = 'online_creator_tester'

import rospy
import sys
import unittest
from roslib.packages import find_resource
from hrsi_representation.msg import QTCArray
import subprocess
import json
import os


class TestOnlineQTC(unittest.TestCase):
    RESULTS_FILE = find_resource(PKG, 'correct.txt')[0]
    BAG_FILE = find_resource(PKG, 'test.bag')[0]

    def __init__(self, *args):
        super(TestOnlineQTC, self).__init__(*args)

        rospy.init_node(NAME)

        self.correct = self._load_file()
        #print self.correct
        self.test = []
        self.seq = 0

    def _load_file(self):
        return json.load(file(self.RESULTS_FILE))

    def _callback(self, msg):
        self.seq = msg.header.seq
        if msg.qtc:
            self.test.append(json.loads(msg.qtc[0].qtc_serialised))

    def test_online_creator(self):
        rospy.Subscriber("/online_qtc_creator/qtc_array", QTCArray, callback=self._callback)
        with open(os.devnull, 'w') as FNULL:
            p = subprocess.Popen("rosbag play -r 2 "+self.BAG_FILE, stdin=subprocess.PIPE, shell=True, stdout=FNULL)
            while p.poll() == None:
                rospy.sleep(1)
        #print self.test
        self.assertEqual(self.test, self.correct)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, TestOnlineQTC, sys.argv)
#    p = TestOnlineQTC(sys.argv)
#    p.test_online_creator()
