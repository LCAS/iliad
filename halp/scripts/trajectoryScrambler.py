#!/usr/bin/env python

'''
Connects to trajectory topic and report 
and publishes next goal in robot plan (geometry_msgs::PoseStamped)
'''

import rospy
from geometry_msgs.msg import PoseStamped
from orunav_msgs.msg import ControllerTrajectoryChunkVec, ControllerTrajectoryChunk, ControllerCommand
import tf
from threading import Lock



class Scrambler():

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
        self.idle = True
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [TrajectoryScrambler] STARTED")
    
        rospy.spin()
    
    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4) 
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'world') 

        self.in_command_topic = rospy.get_param('~in_command_topic', '/robot'+str(self.robot_id)+'/control/controller/commands')
        self.out_command_topic = rospy.get_param('~out_command_topic', '/robot'+str(self.robot_id)+'/control/controller/commands_mpc')

        self.in_trajectory_topic = rospy.get_param('~in_trajectory_topic', '/robot'+str(self.robot_id)+'/control/controller/trajectories')
        self.out_trajectory_topic = rospy.get_param('~out_trajectory_topic', '/robot'+str(self.robot_id)+'/control/controller/trajectories_mpc')

        self.reports_topic = rospy.get_param(
            '~reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports')

    def initROS(self):
        self.trajectory_pub = rospy.Publisher(self.out_trajectory_topic, ControllerTrajectoryChunkVec, queue_size=1)
        self.command_pub = rospy.Publisher(self.out_command_topic, ControllerCommand, queue_size=1)

        rospy.Subscriber(self.in_trajectory_topic, ControllerTrajectoryChunkVec, self.trajectory_callback, queue_size=1)
        rospy.Subscriber(self.in_command_topic, ControllerCommand, self.command_callback, queue_size=1)
        rospy.Subscriber(self.reports_topic, ControllerReport,
                         self.reports_callback, queue_size=1)


    def reports_callback(self, msg):
        if msg.status ==1:
            self.idle = True
        else
            self.idle = False


    def command_callback(self,msg):     

        rospy.loginfo("Node [TrajectoryScrambler] command received. Just for the record")
        self.commands.append(msg)
        

    def trajectory_callback(self,msg):     

        rospy.loginfo("Node [TrajectoryScrambler] trajectory received")
        self.trajectories = msg
        '''
        Let's divide 1 trajectory with N chunks into N trajectories with 1 chunk
        '''
        trajCV = ControllerTrajectoryChunkVec()        

        for chunk in self.trajectories.chunks:              
            newChunk= chunk
            newChunk.sequence_num=0
            newChunk.traj_id=self.local_traj_id
            self.local_traj_id = self.local_traj_id + 1 
            newChunk.final=True

            # Add random constraints
            #newChunk.constraints.bounds_steering_velocity=[-1,1]
            #newChunk.constraints.bounds_tangential_velocity=[-0.005,0.005]

            trajCV.chunks=[]
            trajCV.chunks.append(newChunk)
            self.transmitCommands(newChunk.traj_id)
            self.retransmitTraj(trajCV)                            
            rospy.sleep(2.5)       
            # instead of waiting I should wait for a report or other hint that trajectory has been done


    def transmitCommands(self,traj_id):
        comm = ControllerCommand()
        comm.robot_id = self.robot_id
        comm.command = comm.COMMAND_ACTIVATE
        comm.traj_id = traj_id
        self.command_pub.publish(comm)
        rospy.loginfo("Node [TrajectoryScrambler] activate sent")

        comm.command = comm.COMMAND_STARTTIME
        comm.start_time = rospy.Time.now()
        self.command_pub.publish(comm)
        rospy.loginfo("Node [TrajectoryScrambler] start time sent")
        
    def retransmitTraj(self,chunkVec):
        self.trajectory_pub.publish(chunkVec)
        
        ids=[]
        for chunk in chunkVec.chunks:  
            ids.append( str(chunk.traj_id)+', '+str(chunk.sequence_num) )
        ids=','.join(ids)
        rospy.loginfo("Node [TrajectoryScrambler] retransmitted (traj,seq):"+ids)


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
        rospy.init_node('TrajectoryScrambler')#, log_level=rospy.DEBUG)
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=Scrambler()
        except rospy.ROSInterruptException:
            pass

