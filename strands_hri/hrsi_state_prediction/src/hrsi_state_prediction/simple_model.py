# -*- coding: utf-8 -*-
"""
Created on Tue Aug 11 17:09:06 2015

@author: cdondrup
"""

from abc import ABCMeta
import numpy as np


class SimpleModel(object):
    __metaclass__ = ABCMeta

    _state_chain = []

    def __init__(self):
        self.previous_state = None

    def predict(self, current_state, distance):
        current_state = np.array(current_state)
        current_state = map(int,current_state[np.where(~np.isnan(current_state))])
        for i, f in enumerate(self._state_chain):
            s = f(current_state, distance)
            if s:
                self.previous_state = s
                break
        return self.previous_state

class QTCBPassBy(SimpleModel):
    _state_chain = [
        lambda x,y: ['?',x[1]] if x[1] ==  0 and y in ("pub", "soc", "und") else None,
        lambda x,y: [ -1,x[1]] if x[1] == -1 and y in ("pub",) else None,
        lambda x,y: [  0,x[1]] if x[1] == -1 and y in ("soc", "per", "int") else None,
        lambda x,y: [  0,x[1]] if x[1] ==  0 and y in ("per", "int") else None,
        lambda x,y: [  0,x[1]] if x[1] ==  1 and y in ("per", "int") else None,
        lambda x,y: [  1,x[1]] if x[1] ==  1 and y in ("pub", "soc") else None
    ]

    def __init__(self):
        super(self.__class__, self).__init__()
        self.previous_state = ['?',0]


class QTCBPathCrossing(SimpleModel):
    _state_chain = [
        lambda x,y: ['?',x[1]] if x[1] ==  0 and y in ("pub", "und") else None,
        lambda x,y: [ -1,x[1]] if x[1] == -1 and y in ("pub",) else None,
        lambda x,y: [  0,x[1]] if x[1] == -1 and y in ("soc", "per", "int") else None,
        lambda x,y: [  0,x[1]] if x[1] ==  0 and y in ("per", "int", "soc") else None,
        lambda x,y: [  0,x[1]] if x[1] ==  1 and y in ("per", "int", "soc") else None,
        lambda x,y: ['?',x[1]]
    ]

    def __init__(self):
        super(self.__class__, self).__init__()
        self.previous_state = ['?',0]

class QTCCPathCrossing(SimpleModel):
    _state_chain = [
        lambda x,y: [  0,x[1],  0,x[3]] if y in ("soc", "per", "int") and x[1] in (-1,0) else None,
        lambda x,y: [ -1, x[1], 1, x[3]] if y in ("soc", "per", "int") and x[1] in (1,) and x[3] in (1,) else None,
        lambda x,y: [  0, x[1], 1, x[3]] if y in ("soc", "per", "int") and x[1] == 1 and x[3] == 0 else None,
        lambda x,y: [  1, x[1], 1, x[3]] if y in ("soc", "per", "int") and x[1] == 1 and x[3] == -1 else None,
        lambda x,y: ['?',x[1], '?', x[3]] if x[1] in (1,0) and y in ("pub", "und") else None
    ]

    def __init__(self):
        super(self.__class__, self).__init__()
        self.previous_state = ['?',0, '?', 0]

class QTCBCPathCrossing(SimpleModel):
    _state_chain = [
        lambda x,y: [ -1,x[1]] if x[1] == -1 and len(x) == 2 else None,
        lambda x,y: [  0,x[1],  0,x[3]] if len(x) == 4 and x[1] in (-1,0) else None,
        lambda x,y: [ -1, x[1], 1, x[3]] if len(x) == 4 and x[1] in (1,) and x[3] in (1,) else None,
        lambda x,y: [  0, x[1], 1, x[3]] if len(x) == 4 and x[1] == 1 and x[3] == 0 else None,
        lambda x,y: [  1, x[1], 1, x[3]] if len(x) == 4 and x[1] == 1 and x[3] == -1 else None,
        lambda x,y: ['?',x[1]] if x[1] in (1,0) and len(x) == 2 else None
    ]

    def __init__(self):
        super(self.__class__, self).__init__()
        self.previous_state = ['?',0]


#class QTCCPassBy(SimpleModel):
#    _state_chain = [
#
##        lambda x,y: [ -1,x[1],  0,x[3]] if x[1] == -1 and y in ("pub",) else None,
#        lambda x,y: [ -1,x[1], 1,x[3]] if x[1] == -1 and y in ("pub", "soc", "per") else None,
#        lambda x,y: [  0,x[1], 1,x[3]] if x[1] ==  0 and y in ("soc", "per", "int") else None,
#        lambda x,y: [  1,x[1],  1,x[3]] if x[1] ==  1 and y in ("int",) else None,
#        lambda x,y: [  1,x[1],  0,x[3]] if x[1] ==  1 and y in ("per",) else None,
#        lambda x,y: ['?',x[1],'?',x[3]]
#    ]
#
#    def __init__(self):
#        super(self.__class__, self).__init__()
#        self.previous_state = ['?',0,'?',0]


class QTCCPassBy(SimpleModel):
    _state_chain = [
        lambda x,y: [  0, x[1], 1, x[3]] if y in ("pub", "soc", "per", "int") and x[1] == 0 and x[3] == 1 else None,
        lambda x,y: [ -1, x[1], 1, x[3]] if y in ("pub", "soc", "per", "int") and x[1] in (-1,) and x[3] in (0,) else None,
        lambda x,y: [ -1, x[1], x[2], x[3]] if y in ("pub", "soc", "per", "int") and x[1] in (-1,) and x[3] in (0,) and x[2] in (-1,1) else None,
        lambda x,y: [ -1, x[1], x[3], x[3]] if y in ("pub", "soc", "per", "int") and x[1] == -1 and x[3] in (1,) else None,
        lambda x,y: [ -1, x[1], -1 if x[3] == -1 else 1, x[3]] if y in ("pub", "soc", "per", "int") and x[1] in (0,) and x[3] in (-1,1) else None,
        lambda x,y: [x[1], x[1], 1, x[3]] if y in ("pub", "soc", "per", "int") and x[1] == 1 else None,
        lambda x,y: ['?', x[1], '?', x[3]] if y in ("und",) else None
    ]

    def __init__(self):
        super(self.__class__, self).__init__()
        self.previous_state = ['?',0,'?',0]


#class QTCBCPassBy(SimpleModel):
#    _state_chain = [
#        lambda x,y: ['?',x[1]] if x[1] ==  0 and y in ("pub", "soc", "und") else None,
#        lambda x,y: [ -1,x[1]] if x[1] == -1 and len(x) == 2 else None,
#        lambda x,y: [ -1,x[1], -1,x[3]] if len(x) == 4 and x[1] == -1 else None,
#        lambda x,y: [  0,x[1], -1,x[3]] if len(x) == 4 and x[1] == 0 else None,
#        lambda x,y: [  1,x[1],  -1,x[3]] if len(x) == 4 and x[1] == 1 else None,
#        lambda x,y: [  1,x[1]] if x[1] ==  1 and len(x) == 2 else None
#    ]
#
#    def __init__(self):
#        super(self.__class__, self).__init__()
#        self.previous_state = ['?',0]

class QTCBCPassBy(SimpleModel):
    _state_chain = [
        lambda x,y: ['?', x[1]] if len(x) < 4 and x[1] in (0,1) else None,
        lambda x,y: [ -1, x[1]] if len(x) < 4 and x[1] in (-1,) else None,
        lambda x,y: [  0, x[1], 1, x[3]] if len(x) == 4 and x[1] == 0 and x[3] == 1 else None,
        lambda x,y: [ -1, x[1], 1, x[3]] if len(x) == 4 and x[1] in (-1,) and x[3] in (0,) else None,
        lambda x,y: [ -1, x[1], x[2], x[3]] if len(x) == 4 and x[1] in (-1,) and x[3] in (0,) and x[2] in (-1,1) else None,
        lambda x,y: [ -1, x[1], x[3], x[3]] if len(x) == 4 and x[1] == -1 and x[3] in (1,) else None,
        lambda x,y: [ -1, x[1], -1 if x[3] == -1 else 1, x[3]] if len(x) == 4 and x[1] in (0,) and x[3] in (-1,1) else None,
        lambda x,y: [x[1], x[1], 1, x[3]] if len(x) == 4 and x[1] == 1 else None
    ]

    def __init__(self):
        super(self.__class__, self).__init__()
        self.previous_state = ['?',0]
