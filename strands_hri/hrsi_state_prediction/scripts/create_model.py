#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import argparse
import os
from pprint import pprint
import json
import hrsi_state_prediction.qtc_utils as qu
from qsrrep_ros.ros_client import ROSClient
from qsrrep_lib.rep_io import HMMRepRequestCreate
from copy import deepcopy

def load_files(path):
    ret = []
    for f in os.listdir(path):
        if f.endswith(".txt"):
            print "Reading file %s" % f
            filename = path + '/' + f
            with open(filename, 'r') as qtc:
                ret.append(np.loadtxt(qtc))

    return ret

def create_model(data, pseudo):
    ret = {}
    qsr_seq = []
    for entry in data:
        qsr = deepcopy(entry[:,:4])
        qsr[qsr==qu.NO_STATE] = np.nan
        qsr_seq.append(qsr.tolist())
        for e in entry:
            key = ','.join(map(str,map(int,e[:4])))
            if not key in ret:
                ret[key] = {}
            subkey = ','.join(map(str,map(int,e[4:])))
            if not subkey in ret[key]:
                ret[key][subkey] = 0
            ret[key][subkey] += 1

    for k in ret.keys():
        n = float(np.sum(ret[k].values()))
        for l in ret[k].keys():
            ret[k][l] = float(ret[k][l]) / n

#        for l in qu.create_states('robot'):
#            l = qu.nan_to_no_state(l)
#            print l
#            l = ','.join(map(str,map(int,l)))
#            print l
#            print 1. / n+1
#            if l in ret[k]:
#                ret[k][l] += 1. / (n+1)
#            else:
#                ret[k][l] = 1. / (n+1)

    return ret, qsr_seq

def create_hmm(qsr_seq, filename):
    r = ROSClient()
    q, d = r.call_service(
        HMMRepRequestCreate(
            qsr_seq=qsr_seq,
            qsr_type="qtch"
        )
    )
    with open(filename+".hmm", 'w') as f: f.write(d)

def write_model(model, filename):
    with open(filename+".rules", 'w') as f:
        json.dump(model, f)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-path', help="a path containing the data files", type=str, required=True)
    parser.add_argument('-o', '--output', help="filename for the resulting model without a suffix", type=str, required=True)
    parser.add_argument('--pseudo', help="create pseudo states and transitions", type=bool, default=False)
    args = parser.parse_args()
    model, qsr_seq = create_model(load_files(args.path), args.pseudo)
    pprint(model)
    write_model(model, args.output)
    create_hmm(qsr_seq, args.output)
