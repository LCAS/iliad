#!/usr/bin/env python

'''
Connects to trajectory topic and report 
and publishes next goal in robot plan (geometry_msgs::PoseStamped)
'''

import rospy
from geometry_msgs.msg import PoseStamped
from orunav_msgs.msg import ControllerTrajectoryChunkVec, ControllerReport, ControllerTrajectoryChunk, ControllerCommand, ControllerConstraints
from std_msgs.msg import Float64
import tf
from threading import Lock



class EnvelopeManager():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params        
        self.traj_mutex = Lock()
        self.commands = []
        self.local_traj_id = 0
        self.curr_chunk = -1
        self.idle = True
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name()+ "] STARTED")
    
        rospy.spin()
    
    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4) 
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'world') 
        
        self.tangential_velocity_topic = rospy.get_param('~bounds_tangential_velocity_topic', '/robot'+str(self.robot_id)+'/control/controller/bounds_tangential_velocity')

        self.in_command_topic = rospy.get_param('~in_command_topic', '/robot'+str(self.robot_id)+'/control/controller/commands')
        self.out_command_topic = rospy.get_param('~out_command_topic', '/robot'+str(self.robot_id)+'/control/controller/commands_mpc')

        self.in_trajectory_topic = rospy.get_param('~in_trajectory_topic', '/robot'+str(self.robot_id)+'/control/controller/trajectories')
        self.out_trajectory_topic = rospy.get_param('~out_trajectory_topic', '/robot'+str(self.robot_id)+'/control/controller/trajectories_mpc')

        self.in_reports_topic = rospy.get_param(
            '~reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports_mpc')
        self.out_reports_topic = rospy.get_param(
            '~reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports')

    def initROS(self):
        self.trajectory_pub = rospy.Publisher(self.out_trajectory_topic, ControllerTrajectoryChunkVec, queue_size=1)
        self.command_pub = rospy.Publisher(self.out_command_topic, ControllerCommand, queue_size=1)
        self.reports_pub = rospy.Publisher(self.out_reports_topic, ControllerReport, queue_size=1)

        rospy.Subscriber(self.in_trajectory_topic, ControllerTrajectoryChunkVec, self.trajectory_callback, queue_size=1)
        rospy.Subscriber(self.in_command_topic, ControllerCommand, self.command_callback, queue_size=1)
        rospy.Subscriber(self.in_reports_topic, ControllerReport, self.reports_callback, queue_size=1)
        rospy.Subscriber(self.tangential_velocity_topic, Float64, self.tangential_velocity_callback, queue_size=1)
    # .............................................................................................................

    def tangential_velocity_callback(self, msg):
        self.tangential_velocity = abs(msg.data)

    def reports_callback(self, msg):
        # you need it to be idle. Using 4 (finishing) didn't work well. 
        if (msg.status == 1):
            self.sendNext()
        else:
            self.idle = False

        self.translateChunks(msg)
        self.reports_pub.publish(msg)

    def translateChunks(self, msg):
        local_chunk_ind = msg.traj_chunk_sequence_num
        local_step_ind = msg.traj_step_sequence_num
        

        global_chunk_ind = max(self.curr_chunk -1,0)
        global_step_ind = msg.traj_step_sequence_num

        msg.traj_chunk_sequence_num = int(global_chunk_ind) 
        msg.traj_step_sequence_num =int(global_step_ind) 

        
    def command_callback(self,msg):     

        rospy.loginfo("Node [" + rospy.get_name()+ "] command received. Just for the record")
        self.commands.append(msg)
        

    def trajectory_callback(self,msg):     
        rospy.loginfo("Node [" + rospy.get_name()+ "] trajectory received")
        self.trajectories = msg
        self.curr_chunk = 0

    def sendNext(self):
        '''
        Let's divide 1 trajectory with N chunks into N trajectories with 1 chunk
        '''
        # do we have anything to transmit?
        if (self.curr_chunk>=0):
            # do we have any chunk to transmit?
            if self.curr_chunk<len(self.trajectories.chunks):
                # index to my chunk list    
                chunk = self.trajectories.chunks[self.curr_chunk]
                self.curr_chunk = self.curr_chunk +1 
                
                
                chunk.sequence_num=0
                chunk.traj_id=self.local_traj_id
                self.local_traj_id = self.local_traj_id + 1 
                chunk.final=True

                # constraints
                chunk.constraints = self.getCurrentConstraints(chunk.constraints)
                
                trajCV = ControllerTrajectoryChunkVec()    
                trajCV.chunks=[]
                trajCV.chunks.append(chunk)
                self.transmitCommands(chunk.traj_id)
                self.retransmitTraj(trajCV)                            
            # finished transmitting...
            else:
                self.curr_chunk = -1 

    def getCurrentConstraints(self,myConstraints):

                
        # All three arrays must be of the same length.
        # Arrays can be empty.
        '''
        THIS DOES NOT WORK?????
        myConstraints.spatial_coef_a0 = [ -1,  0]
        myConstraints.spatial_coef_a1 = [  0, -1]
        myConstraints.spatial_coef_b  = [  1,  5]
        '''


        # Two numbers: minimal and maximal bounds // or an empty array]
        # myConstraints.bounds_orientation =[ config["bounds_orientation_min"], config["bounds_orientation_max"]]            
        # Two numbers: minimal and maximal bounds // or an empty array]
        # myConstraints.bounds_steering_velocity =[ config["bounds_steering_velocity_min"], config["bounds_steering_velocity_max"]]            
        # Two numbers: minimal and maximal bounds // or an empty array]
        # myConstraints.bounds_tangential_velocity =[ config["bounds_tangential_velocity_min"], config["bounds_tangential_velocity_max"]]            
        # Two numbers: minimal and maximal bounds // or an empty array]
        # myConstraints.bounds_tangential_acceleration =[ config["bounds_tangential_acceleration_min"], config["bounds_tangential_acceleration_max"]]            
        # One number or an empty array
        # myConstraints.bounds_centripetal_acceleration = config["bounds_centripetal_acceleration_min"]       

        if hasattr(self, 'tangential_velocity'):
            myConstraints.bounds_tangential_velocity =[ -self.tangential_velocity, self.tangential_velocity ]                     

        return myConstraints



    def transmitCommands(self,traj_id):
        comm = ControllerCommand()
        comm.robot_id = self.robot_id
        comm.command = comm.COMMAND_ACTIVATE
        comm.traj_id = traj_id
        self.command_pub.publish(comm)
        rospy.loginfo("Node [" + rospy.get_name()+ "] activate sent")

        comm.command = comm.COMMAND_STARTTIME
        comm.start_time = rospy.Time.now()
        self.command_pub.publish(comm)
        rospy.loginfo("Node [" + rospy.get_name()+ "] start time sent")
        
    def retransmitTraj(self,chunkVec):
        self.trajectory_pub.publish(chunkVec)
        
        ids=[]
        for chunk in chunkVec.chunks:  
            ids.append( str(chunk.traj_id)+', '+str(chunk.sequence_num) )
        ids=','.join(ids)
        rospy.loginfo("Node [" + rospy.get_name()+ "] retransmitted (traj,seq):"+ids)


    def modulateStep(self,traject_ChV):
        '''
        takes a full chunk vector and mingles with its positions/speeds
        controller accepts it
        '''
        for chunk in traject_ChV.chunks:        
            for step in chunk.steps:
                 step.velocities.tangential = 2.0 * step.velocities.tangential
                 step.velocities.steering  = 2.0 * step.velocities.steering 
                 step.state.position_x = 0.5 + step.state.position_x
                 step.state.position_y =  0.5 + step.state.position_y

        self.retransmit(self.trajectories)



# Main function.
if __name__ == '__main__':
        rospy.init_node('EnvelopeManager')#, log_level=rospy.DEBUG)
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=EnvelopeManager()
        except rospy.ROSInterruptException:
            pass

