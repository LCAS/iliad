#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 19 09:01:18 2016

@author: cd32
"""

from __future__ import print_function
import argparse
import yaml
from pymongo import MongoClient
import os
import csv
from collections import OrderedDict


class CreateCSV(object):
    def __init__(self, config_file, save_path, dbhost, dbport, db_name,
                 data_collection, annotation_collection):
        print("Connecting")
        client = MongoClient(dbhost, dbport)
        db = client[db_name]
        collections = db.collection_names(include_system_collections=False)
#        print(collections)
        if data_collection not in collections:
            raise KeyError("Collection '%s' could not be found in db '%s'." % (data_collection, db_name))
        if annotation_collection not in collections:
            raise KeyError("Collection '%s' could not be found in db '%s'." % (annotation_collection, db_name))

        print("Creating index.")
        db[data_collection].ensure_index("uuids")

        classes = self.parse_yaml(config_file)["buttons"]
        for c in classes:
            c = c.replace(' ','-')
            print("######### %s #########" % c)
            for uuid, data in self.parse_db(db[data_collection], db[annotation_collection], c):
                self.save_file(c, uuid, data, save_path)

    def parse_yaml(self, f):
        with open(f, 'r') as stream:
            try:
                return yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)

    def parse_db(self, data_collection, annotation_collection, class_name):
        for x in annotation_collection.find({"class": class_name}):
            poses = []
            for y in data_collection.find({"uuids": x["uuid"]}):
                stamp = y["header"]["stamp"]
                person = y["people"][y["uuids"].index(x["uuid"])]["pose"]
                res = OrderedDict() # Increase readability of resulting file
                res["time"] = float(str(stamp["secs"])+"."+str(stamp["nsecs"]))
                res["agent1"] = "human"
                res["x1"] = person["position"]["x"]
                res["y1"] = person["position"]["y"]
                res["agent2"] = "robot"
                res["x2"] = y["robot"]["position"]["x"]
                res["y2"] = y["robot"]["position"]["y"]
                poses.append(res)
            yield x["uuid"], sorted(poses, key=lambda x: x["time"])

    def save_file(self, class_name, uuid, data, save_path):
        out = save_path+"/"+class_name
        if not os.path.exists(out):
            os.makedirs(out)
        f = out+"/"+uuid+".csv"
        with open(f, 'w') as csvfile:
            print("Writing '%s' to: %s" % (uuid, f))
            fieldnames = data[0].keys()
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for d in data:
                writer.writerow(d)


if __name__=="__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=str,
                        help="Path to yaml config file")
    parser.add_argument("save_path", type=str,
                        help="Path to save the csv files under. Naming scheme: uuid.csv. Creates directories for each class found.")
    parser.add_argument("--dbhost", type=str, default='localhost',
                        help="The database host IP.")
    parser.add_argument("--dbport", type=int, default=62345,
                        help="The database port")
    parser.add_argument("--db", type=str, default="message_store",
                        help="The database name")
    parser.add_argument("--data_collection", type=str, default="people_perception",
                        help="The collection in the database db that contains the people tracks")
    parser.add_argument("--annotation_collection", type=str, default="people_perception_annotations",
                        help="The collection in the database db that contains the results")
    args = parser.parse_args()

    c = CreateCSV(
        config_file=args.config,
        save_path=args.save_path,
        dbhost=args.dbhost,
        dbport=args.dbport,
        db_name=args.db,
        data_collection=args.data_collection,
        annotation_collection=args.annotation_collection
    )

