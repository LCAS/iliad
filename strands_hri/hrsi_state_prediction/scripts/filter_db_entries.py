#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 17 10:46:31 2016

@author: cdondrup
"""

from __future__ import print_function
import math
import argparse
from pymongo import MongoClient
import numpy
import yaml


class Filter(object):
    __uuid = "uuid"
    __count = "count"
    __min_dist = "min_dist"
    __travel_dist = "travel_dist"
    __last_pose = "last_pose"
    __ids = "ids"
    __class = "class"

    def __init__(self, host, port, db_name, data_collection, filtered_collection,
                 min_num_entries, min_min_dist, min_travel_dist, force, save, load):

        print("Connecting")
        client = MongoClient(host, port)
        db = client[db_name]
        collections = db.collection_names(include_system_collections=False)
        if data_collection not in collections:
            raise KeyError("Collection '%s' could not be found in db '%s'." % (data_collection, db_name))

        if filtered_collection in collections:
            if force:
                db[filtered_collection].drop()
                print("Old collection '%s' has been dropped." % filtered_collection)
            else:
                print("Collection '%s' found. New results will be added. Please restart with -f to force override." % filtered_collection)

        self.prepare(db, data_collection, "uuids")

        if load != "":
            people = self.load_yaml(load)
        else:
            people = self.get_uuids(db, data_collection)
        print("Found %s people" % len(people))
        if save != "":
            self.save_yaml(people, save)

        print("Filtering data collection '%s'." % data_collection)
        db.create_collection(filtered_collection)
        try:
            self.parse_db(
                uuids=people,
                data_collection=db[data_collection],
                filtered_collection=db[filtered_collection],
                min_min_dist=min_min_dist,
                min_num_entries=min_num_entries,
                min_travel_dist=min_travel_dist
            )
        except KeyboardInterrupt:
            print("Action terminated. Good-bye.")
        finally:
            self.prepare(db, filtered_collection, "uuid")
        print("Done")

    def load_yaml(self, f):
        with open(f, 'r') as stream:
            try:
                print("Loading from %s" % f)
                return yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)

    def save_yaml(self, d, f):
        with open(f, 'w') as stream:
            try:
                print("Saving to %s" % f)
                return yaml.dump(d, stream)
            except yaml.YAMLError as exc:
                print(exc)

    def prepare(self, db, collection, index_key):
        pp = db[collection]
        cnt = pp.count()
        print("Found %s entries" % cnt)
        if cnt == 0:
            raise KeyError("No entries in %s" % collection)
        print("Creating index over '%s' for collection '%s'." % (index_key,collection))
        pp.ensure_index(index_key)

    def get_uuids(self, db, collection):
        pp = db[collection]
        print("Getting list of uuids. This might take a while.")
        uuids = pp.distinct("uuids")
        return uuids

    def euclidean_distance(self, human, robot):
        return numpy.sqrt(
            numpy.power(
                human["x"]-robot["x"],
                2
            ) + numpy.power(
                human["y"]-robot["y"],
                2
            )
        )

    def parse_db(self, uuids, data_collection, filtered_collection, min_num_entries,
                 min_min_dist, min_travel_dist):
        cnt = 0
        for i, uuid in enumerate(uuids):
            print("Processing %s/%s entry. %s %% complete. People inserted: %s" % (i, len(uuids), math.ceil(float(i)/float(len(uuids))*100.), cnt), end='\r')
            elem = data_collection.find({"uuids": uuid})
            if elem.count() < min_num_entries:
                continue
            poses = []
            dist_h = 0.
            dist_r = 0.
            min_dist = 1000.
            for x in elem:
                stamp = x["header"]["stamp"]
                person = x["people"][x["uuids"].index(uuid)]["pose"]
                res = {
                    "uuid": uuid,
                    "time": float(str(stamp["secs"])+"."+str(stamp["nsecs"])),
                    "human": {"x": person["position"]["x"], "y": person["position"]["y"]},
                    "robot": {"x": x["robot"]["position"]["x"], "y":x["robot"]["position"]["y"]}
                }
                poses.append(res)
                try:
                    dist_h += self.euclidean_distance(poses[-2]["human"], poses[-1]["human"])
                    dist_r += self.euclidean_distance(poses[-2]["robot"], poses[-1]["robot"])
                except IndexError:
                    pass
                cd = self.euclidean_distance(poses[-1]["human"], poses[-1]["robot"])
                min_dist = cd if cd < min_dist else min_dist
            if min_dist > min_min_dist or dist_h < min_travel_dist or dist_r < min_travel_dist:
                continue
#            print("Called")
            filtered_collection.insert(sorted(poses, key=lambda x: x["time"]))
            cnt += 1

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--force", action='store_true',
                        help="If collection already exists, it will be dropped first to force override.")
    parser.add_argument("-s", "--save_uuids", type=str, default="",
                        help="Specify a file to save the list of distinct uuids in. Yaml format.")
    parser.add_argument("-l", "--load_uuids", type=str, default="",
                        help="Specify a file to load the list of distinct uuids from to prevent parsing the DB. Yaml format.")
    parser.add_argument("--dbhost", type=str, default='localhost',
                        help="The database host IP.")
    parser.add_argument("--dbport", type=int, default=62345,
                        help="The database port")
    parser.add_argument("--db", type=str, default="message_store",
                        help="The database name")
    parser.add_argument("--data_collection", type=str, default="people_perception",
                        help="The collection in the database db that contains the people tracks")
    parser.add_argument("--filtered_collection", type=str, default="people_perception_filtered",
                        help="The collection in the database db that should contain the results")
    parser.add_argument("--min_num_entries", type=int, default=100,
                        help="The minimum number of entries per uuid to be considered for annotation")
    parser.add_argument("--min_min_dist", type=float, default=2.,
                        help="The minimum minimum distance of the human to the robot for a uuid to be considered for annotation")
    parser.add_argument("--min_travel_dist", type=float, default=3.,
                        help="The minimum travel distance accumulated over all poses of human and robot for a uuid to be considered for annotation")
    args = parser.parse_args()

    app = Filter(
        host=args.dbhost,
        port=args.dbport,
        db_name=args.db,
        data_collection=args.data_collection,
        filtered_collection=args.filtered_collection,
        min_num_entries=args.min_num_entries,
        min_min_dist=args.min_min_dist,
        min_travel_dist=args.min_travel_dist,
        force=args.force,
        save=args.save_uuids,
        load=args.load_uuids
    )
