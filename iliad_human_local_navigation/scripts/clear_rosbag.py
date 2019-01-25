#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''

Parses a rosbag and removes timestamps from headers in topics.
Might be useful for looping playbacks.

'''

import rosbag
import sys
import getopt


def clear_timestamp(ob):
    if hasattr(ob, 'header'):
        ob.header.stamp.secs = 0
        ob.header.stamp.nsecs = 0

    if hasattr(ob, '__dict__'):
        att = ob.__dict__.keys()
        for a in att:
            setattr(ob, a, clear_timestamp(getattr(ob, a)))

    elif hasattr(ob, '__slots__'):
        att = ob.__slots__
        for a in att:
            setattr(ob, a, clear_timestamp(getattr(ob, a)))

    elif type(ob) is list:
        for index in range(len(ob)):
            ob[index] = clear_timestamp(ob[index])

    return ob


def main(argv):
    inBagFileName = '/home/manolofc/iliad/BAGS-Tejas/S1-T1.1-A1.bag'
    outBagFileName = '/home/manolofc/iliad/BAGS-Tejas/S1-T1.1-A1-timeless.bag'

    try:
        opts, args = getopt.getopt(argv, "hi:o:", ["ifile=", "ofile="])
    except getopt.GetoptError:
        print 'clear_rosbag.py -i <inputfile> -o <outputfile>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'clear_rosbag.py -i <inputfile> -o <outputfile>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inBagFileName = arg
        elif opt in ("-o", "--ofile"):
            outBagFileName = arg
    print 'Input file is ', inBagFileName
    print 'Output file is ', outBagFileName

    with rosbag.Bag(outBagFileName, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inBagFileName).read_messages():
            msg = clear_timestamp(msg)
            # if topic == "/tf":
            #print type(msg.transforms)
            #print attr(msg, '__dict__')
            #print msg
            outbag.write(topic, msg, t)
    print 'Finished'


if __name__ == "__main__":
    main(sys.argv[1:])
