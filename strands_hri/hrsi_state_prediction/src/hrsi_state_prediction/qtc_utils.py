# -*- coding: utf-8 -*-
"""
Created on Fri Aug 21 15:47:20 2015

@author: cdondrup
"""

import numpy as np
import rospy


NO_STATE = 9.

def create_states(qtc_type):
    try:
        if qtc_type == "":
            raise AttributeError()
        elif qtc_type == 'qtcbs':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    yield [i-2, j-2]
        elif qtc_type is 'qtccs':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            yield [i-2, j-2, k-2, l-2]
        elif qtc_type.startswith('qtcbcs'):
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    yield [i-2, j-2, np.NaN, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            yield [i-2, j-2, k-2, l-2]
        elif qtc_type is 'qtch':
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        yield [i-2, j-2, k-2, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    for k in xrange(1, 4):
                        for l in xrange(1, 4):
                            yield [i-2, j-2, k-2, l-2]
        elif qtc_type is 'robot':
            for i in xrange(1, 4):
                yield [i-2, np.NaN]
            for i in xrange(1, 4):
                for j in xrange(1, 4):
                    yield [i-2, j-2]

    except AttributeError:
        rospy.logfatal("QTC type: %s not found" % qtc_type)

def to_symbol(qtc_data):
    qtc_data = np.array(qtc_data)
    d = len(qtc_data)
    mult = 3**np.arange(d-1, -1, -1)
    return ((qtc_data + 1)*mult).sum() + 1

def csv_to_array(s):
    return np.array(map(int,s.split(',')))

def filter_no_state(a):
    a = np.array(a)
    return a[a!=NO_STATE]

def nan_to_no_state(a):
    a = np.array(a)
    a[np.isnan(a)] = NO_STATE
    return a
