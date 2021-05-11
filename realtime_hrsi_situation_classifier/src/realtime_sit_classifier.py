#!/usr/bin/python3.5
import roslibpy
import hmms
import time
import numpy as np
import os
from itertools import combinations
from scipy.stats import entropy
import sys

class situation_prediction():
    def __init__(self,robot_id):
        # init HMM model
        working_dir = os.path.dirname(os.path.realpath(__file__))

        # Load per-situation HMMs
        self.pblHMM = hmms.DtHMM.from_file(working_dir+"/../Models/pblHMM_k_3_hf.npz")
        self.pbrHMM = hmms.DtHMM.from_file(working_dir+"/../Models/pbrHMM_k_3_hf.npz")
        self.rotlHMM = hmms.DtHMM.from_file(working_dir+"/../Models/rotlHMM_k_3_hf.npz")
        self.rotrHMM = hmms.DtHMM.from_file(working_dir+"/../Models/rotrHMM_k_3_hf.npz")
        self.pclHMM = hmms.DtHMM.from_file(working_dir+"/../Models/pclHMM_k_3_hf.npz")
        self.pcrHMM = hmms.DtHMM.from_file(working_dir+"/../Models/pcrHMM_k_3_hf.npz")

        # Configuration parameters
        self.rejection_KL_thresh = 0.025
        N_nodes = 81

        self.classes = ["PBL", "PBR", "ROTL", "ROTR", "PCL", "PCR"]

        # Functions for mapping QTC_C states to integers
        # Create list of QTC_C states so that indices can be used as integer state IDs compatible with HMM library
        QTC_symbols = []
        for i in range(0, 4):
            QTC_symbols.append("-")
            QTC_symbols.append("0")
            QTC_symbols.append("+")
        # print("QTC symbols:", QTC_symbols[:3])
        self.QTC_C_states = list(combinations(QTC_symbols, 4))
        self.QTC_C_states = [state[0] + state[1] + state[2] + state[3] for state in self.QTC_C_states]
        self.QTC_C_states = list(np.unique(self.QTC_C_states))

        # init param
        self.prev_time = time.time()
        self.qtc_seq = []

        # init ros
        ros_client = roslibpy.Ros(host='localhost', port=9090)
        ros_client.run()
        print("ROS connected:", ros_client.is_connected)
        
        # read parameters
        print("Robot Id :", robot_id)

        rejection_thresh_param = roslibpy.Param(ros_client, "/robot"+str(robot_id)+"/qsr/situation_rejection_threshold")
        self.rejection_KL_thresh = rejection_thresh_param.get()

        print("rejection_KL_thresh:",self.rejection_KL_thresh )
        
        # define topics
        self.qtcTopic = roslibpy.Topic(ros_client, "/robot"+str(robot_id)+"/qsr/qtc_state", "std_msgs/String")
        self.sitTopic = roslibpy.Topic(ros_client, "/robot"+str(robot_id)+"/qsr/situation_predictions", "std_msgs/String")

        # define subscribers
        self.qtcTopic.subscribe(lambda message: self.qtc_update_callback(message))

        while ros_client.is_connected:
            time.sleep(1)

        ros_client.terminate()

    def qtc_update_callback(self,qtc_state_msg):
        
        qtc_state_str = qtc_state_msg['data'].replace(",", "")
        current_time = time.time()
        if current_time - self.prev_time > 3:
            self.qtc_seq = []
            self.sitTopic.publish(roslibpy.Message({'data': "None"}))

        if len(self.qtc_seq) == 0:
            self.qtc_seq.append(qtc_state_str)
        elif qtc_state_str != self.qtc_seq[-1]:
            self.qtc_seq.append(qtc_state_str)

        #print(self.qtc_seq)
        sit = self.classify_QTC_seqs(np.array([self.QTC_C_seq_to_num_seq(self.qtc_seq)]))
        #print(sit, "\n")
        self.sitTopic.publish(roslibpy.Message({'data': sit}))

        self.prev_time = current_time

    def classify_QTC_seqs(self,e_seqs):
        ll_pb_l = self.pblHMM.data_estimate(e_seqs)
        ll_pb_r = self.pbrHMM.data_estimate(e_seqs)
        ll_rotl = self.rotlHMM.data_estimate(e_seqs)
        ll_rotr = self.rotrHMM.data_estimate(e_seqs)
        ll_pcl = self.pclHMM.data_estimate(e_seqs)
        ll_pcr = self.pcrHMM.data_estimate(e_seqs)

        lls = [ll_pb_l, ll_pb_r, ll_rotl, ll_rotr, ll_pcl, ll_pcr]
        class_id = np.argmax(lls)
        KL = entropy(lls, [1 / len(lls) for ll in lls])
        if self.rejection_KL_thresh is None:
            self.rejection_KL_thresh = 0.018

        if KL > self.rejection_KL_thresh:
            pred = self.classes[class_id]
        else:
            pred = "rejection"
            class_id = len(self.classes)
        # print("Classified as", pred)
        # print("KL divergence of likelihoods from uniform distribution:", KL)

        return pred


    def QTC_C_seq_to_num_seq(self,QTC_C_seq):
        num_seq = []
        for QTC_C in QTC_C_seq:
            num_seq.append(self.QTC_C_states.index(QTC_C))

        return num_seq


# Main function.
if __name__ == '__main__':
   robot_id = sys.argv[1]
   sp = situation_prediction(robot_id)
