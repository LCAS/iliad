#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 29 11:07:40 2015

@author: cdondrup
"""
import os
import csv
import copy
import rospy
import numpy as np
from hrsi_representation.input_base_abstractclass import InputBaseAbstractclass


class FileInput(InputBaseAbstractclass):
    """Reads files for training"""

    def __init__(self):
        """Creates a new instance of the FileReader class
        """
        super(FileInput, self).__init__()

    def generate_data_from_input(self, *args, **kwargs):
        """reads all .csv files from a given directory and returns them as
        numpy arrays

        :param path: the directory which contains the csv files

        :return: the dict represetation of all the found files
        """

        ret = []
        files = []

        for f in os.listdir(kwargs["path"]):
            if f.endswith(".csv"):
                files.append(f)
                data = copy.deepcopy(self.template)
                filename = kwargs["path"] + '/' + f
                with open(filename) as csvfile:
                    reader = csv.DictReader(csvfile)
                    rospy.loginfo("Reading file: '%s'" % f)
                    for idx,row in enumerate(reader):
                        if idx % 3 == 0:
                            if data["agent1"]["name"] == "":
                                data["agent1"]["name"] = row[kwargs["k"]]
                            if data["agent2"]["name"] == "":
                                data["agent2"]["name"] = row[kwargs["l"]]

                            data["agent1"]["x"] = np.append(data["agent1"]["x"], float(row[kwargs["k_x"]]))
                            data["agent1"]["y"] = np.append(data["agent1"]["y"], float(row[kwargs["k_y"]]))

                            data["agent2"]["x"] = np.append(data["agent2"]["x"], float(row[kwargs["l_x"]]))
                            data["agent2"]["y"] = np.append(data["agent2"]["y"], float(row[kwargs["l_y"]]))

                    ret.append(data)

        return ret, files
