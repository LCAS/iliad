#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 29 11:36:24 2015

@author: cdondrup
"""

import rospy
import numpy as np
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
from qsrlib.qsrlib import QSRlib_Request_Message
from abc import abstractmethod, ABCMeta
try:
    import cPickle as pickle
except:
    import pickle


class InputBaseAbstractclass(object):
    """Provides functions for:
        - the transformation of raw data into qsr_lib format
        - converting the data into QTC using qsr_lib

    Will be used as a base class for the training and online data input classes
    """

    __metaclass__ = ABCMeta

    def __init__(self):
        """Creates a new instance of the InputBaseClass"""
        # The order of this list has to reflect the index of the qtc_type in the dynamic reconfigure file
        self.qtc_types = [
            "qtcbs",
            "qtccs",
            "qtcbcs",
            "qtcbcs_argprobd"
        ]
        self.argprobd = "argprobd"
        self.template = {
            "agent1": {
                "name": "",
                "x": np.array([]),
                "y": np.array([])
            },
            "agent2": {
                "name": "",
                "x": np.array([]),
                "y": np.array([])
            }
        }
        self.qtc = None

    def _request_qtc(self, qsr, world, parameters):
        qrmsg = QSRlib_Request_Message(
            which_qsr=qsr,
            input_data=world,
            dynamic_args=parameters
        )

        #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Creating client QSRlib")
        cln = QSRlib_ROS_Client()

        #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Create message")
        req = cln.make_ros_request_message(qrmsg)

        #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Sending request")
        res = cln.request_qsrs(req)

        #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Unpickling response")
        out = pickle.loads(res.data)
        qtc = []
        dis = []
        for t in out.qsrs.get_sorted_timestamps():
            for k, v in out.qsrs.trace[t].qsrs.items():
#                print v.qsr.items()
#                if len(v.qsr.items()) < 2:
#                    continue # Hacky but we only want dist when qtc is there too.
                for l, w in v.qsr.items():
                    if l.startswith("qtc"):
#                        q = self._to_np_array(w)
#                        if l.startswith("qtcbcs"):
#                            q = q if len(q) == 4 else np.append(q, [np.nan, np.nan])
#                        qtc = np.array([q]) if not qtc.size else np.append(qtc, [q], axis=0)
                        qtc.append(w)
                    elif l == "argprobd":
                        dis.append(w)

        return qtc, dis

    def _convert_to_world(self, data_dict):
        world = World_Trace()

        agent1 = data_dict["agent1"]
        agent2 = data_dict["agent2"]

        name1 = agent1["name"]
        name2 = agent2["name"]

        x1 = np.array(agent1["x"], dtype=float)
        y1 = np.array(agent1["y"], dtype=float)

        x2 = np.array(agent2["x"], dtype=float)
        y2 = np.array(agent2["y"], dtype=float)

        ob = []
        for idx, (e_x1, e_y1, e_x2, e_y2) in enumerate(zip(x1,y1,x2,y2)):
            ob.append(Object_State(
                name=name1,
                timestamp=idx,
                x=e_x1,
                y=e_y1
            ))
            ob.append(Object_State(
                name=name2,
                timestamp=idx,
                x=e_x2,
                y=e_y2
            ))

        world.add_object_state_series(ob)
        return world

    @abstractmethod
    def generate_data_from_input(self, *args, **kwargs):
        """Input data into the conversion process"""
        pass

    def convert(self, data, qtc_type, parameters, argprobd=True):
        """Convert data inserted via put() into QTC

        :param qtc_type: qtcb|qtcc|qtcbc
        """
        #rospy.logdebug( "[" + rospy.get_name() + "]: " + "converting data")

        data = [data] if not isinstance(data, list) else data
        ret = []
        for elem in np.array(data):
            #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Processing data element")

            world = self._convert_to_world(data_dict=elem)
            #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Converted to world whatever")

            if argprobd:
                qsr = [qtc_type, self.argprobd]
            else:
                qsr = qtc_type

            #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Obtaining qtc")
            requested_qtc = self._request_qtc(qsr=qsr, world=world, parameters=parameters)

            #rospy.logdebug( "[" + rospy.get_name() + "]: " + "Appending")
            ret.append(requested_qtc)

            #rospy.logdebug( "[" + rospy.get_name() + "]: " + "And done")

        return ret

    def _to_np_array(self, string):
        string = string.values()[0] if isinstance(string, dict) else string
        return np.fromstring(string.replace('-','-1').replace('+','+1'), dtype=int, sep=',')
