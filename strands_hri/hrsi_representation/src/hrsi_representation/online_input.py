#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 29 11:07:40 2015

@author: cdondrup
"""
#import os
#import csv
import copy
#import numpy as np
from hrsi_representation.input_base_abstractclass import InputBaseAbstractclass


class OnlineInput(InputBaseAbstractclass):
    """Reads files for training"""

    def __init__(self):
        """Creates a new instance of the FileReader class
        """
        super(OnlineInput, self).__init__()

    def generate_data_from_input(self, *args, **kwargs):
        """reads all .csv files from a given directory and returns them as
        numpy arrays

        :param path: the directory which contains the csv files

        :return: the qtc represetation of all the found files
        """

        data = copy.deepcopy(self.template)
        data["agent1"]["name"] = kwargs["agent1"]
        data["agent2"]["name"] = kwargs["agent2"]
        data["agent1"]["x"] = kwargs["x1"]
        data["agent1"]["y"] = kwargs["y1"]
        data["agent2"]["x"] = kwargs["x2"]
        data["agent2"]["y"] = kwargs["y2"]

        return data
