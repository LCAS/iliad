#!/usr/bin/env python

'''
Connects to trajectory topic and report 
and publishes next goal in robot plan (geometry_msgs::PoseStamped)
'''

import rospy
from geometry_msgs.msg import PoseStamped
from orunav_msgs.msg import ControllerTrajectoryChunkVec, ControllerReport
import tf
from threading import Lock



class retriever():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params        
        self.traj_mutex = Lock()

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [GoalRetriever] STARTED")
    
        rospy.spin()
    
    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4) 
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'world') 

        # how many steps ahead we use for target
        self.look_ahead = rospy.get_param('~look_ahead', 5) 

        self.reports_topic = rospy.get_param('~reports_topic', '/robot4/control/controller/reports') 
        self.trajectory_topic = rospy.get_param('~trajectory_topic', '/robot4/control/controller/trajectories')
        self.goals_topic = rospy.get_param('~goals_topic', '/robot4/control/controller/nextGoal')

    def initROS(self):
        self.goal_pub = rospy.Publisher(self.goals_topic, PoseStamped, queue_size=1)
        rospy.Subscriber(self.trajectory_topic, ControllerTrajectoryChunkVec, self.trajectory_callback, queue_size=1)
        rospy.Subscriber(self.reports_topic, ControllerReport, self.reports_callback, queue_size=1)

    def trajectory_callback(self,msg):     
        '''
        When robot receives a trajectory list, we store it.
        '''
        self.traj_mutex.acquire()   
        self.curr_traj = msg
        self.traj_mutex.release()

    def reports_callback(self,msg):
        
        # if robot is not active, there is no goal
        if msg.status!=3:
            # normal status do nothing
            return

        try:      
            self.traj_mutex.acquire()   
            trajectory = self.curr_traj 
            self.traj_mutex.release()
        except AttributeError:
            rospy.logerr("[GoalRetriever] Don't have a trajectory yet.")
            # in the weird situation where we receive a report before procesing a trajectory callback
            return

        curr_chunk_id = msg.traj_chunk_sequence_num
        curr_step_id = msg.traj_step_sequence_num
        
        # how many chunks do we have in current trajectory?
        max_num_chunks = len(trajectory.chunks)
        
        if curr_chunk_id>=max_num_chunks: 
            rospy.logerr("[GoalRetriever] report points to a chunck outside our saved trajectory")
            return

        # how many steps does have current chunck?
        curr_chunk= trajectory.chunks[curr_chunk_id]
        max_num_steps = len(curr_chunk.steps)        

        # current goal is somewhere among next trajectory steps in current chunck         
        goal_step_id = curr_step_id + self.look_ahead

        # goal might be inside of next chunck, or there might not be any
        while goal_step_id>=max_num_steps:
            # select step in next chunck
            goal_step_id = max_num_steps - goal_step_id
            curr_chunk_id = curr_chunk_id + 1             
            
            if curr_chunk_id>=max_num_chunks:     
                rospy.logerr("[GoalRetriever] No more chunck targets ...")
                return

            # finally we can select step in next chunck
            curr_chunk = trajectory.chunks[curr_chunk_id]
            # and go back to while loop, to test this chunck has enough steps
            max_num_steps = len(curr_chunk.steps)

        try: 
            goal_step = curr_chunk.steps[goal_step_id]
        except IndexError:
            rospy.logerr("[GoalRetriever] No more step targets ...")
            return

        aPoseSt = PoseStamped()
        aPoseSt.header.frame_id = self.goal_frame_id
        aPoseSt.pose.position.x = goal_step.state.position_x
        aPoseSt.pose.position.y = goal_step.state.position_y
        
        quat = tf.transformations.quaternion_from_euler(0,0,goal_step.state.orientation_angle)
        aPoseSt.pose.orientation.x = quat[0]
        aPoseSt.pose.orientation.y = quat[1]
        aPoseSt.pose.orientation.z = quat[2]
        aPoseSt.pose.orientation.w = quat[3]

        self.goal_pub.publish(aPoseSt)



# Main function.
if __name__ == '__main__':
        rospy.init_node('GoalRetriever')#, log_level=rospy.DEBUG)
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=retriever()
        except rospy.ROSInterruptException:
            pass

