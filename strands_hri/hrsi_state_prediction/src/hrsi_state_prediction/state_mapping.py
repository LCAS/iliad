# -*- coding: utf-8 -*-
"""
Created on Wed Sep 23 21:56:12 2015

@author: cdondrup
"""

import rospy
import json
from pprint import pprint
import numpy as np


class StateMapping(object):
    __no_state__ = 9.

    _previous_state = ['?', 0]

    model = None

    def load_model(self, filename):
        with open(filename, 'r') as f:
            self.model =json.load(f)
        pprint(self.model)

    def predict(self, qtc_robot, qtc_goal):
        if self.model == None:
            rospy.logfatal("No model loaded. Please use the load_model service to load a model prior to the interaction.")
            return
        qtc_robot = np.array(qtc_robot[-1]); qtc_goal = np.array(qtc_goal[-1])
        qtc_robot[np.isnan(qtc_robot)] = self.__no_state__
        qtc_goal[np.isnan(qtc_goal)] = self.__no_state__
        qtc = np.append(qtc_goal[[1,3]], qtc_robot[[1,3]])
        qtc = ','.join(map(str,map(int,qtc)))
        print "OBSERVATION", qtc
        try:
            states = self.model[qtc].keys()
            probs = self.model[qtc].values() # Both lists are always in a corresponding order
#            prediction = np.random.choice(states, p=probs)
            prediction = states[probs.index(max(probs))]
            prediction = map(int,prediction.split(','))
            prediction = [prediction[0], int(qtc_robot[1]), prediction[1], int(qtc_robot[3])] \
                if prediction[1] != 9 else [prediction[0], int(qtc_robot[1])]
        except KeyError:
            prediction = self._previous_state
        self._previous_state = prediction
        return prediction