#!/usr/bin/env python

'''

Reads a Twist containing translational vel (y axis) 
and rotational vel (x axis) 

UNFINISHED!!!!!

'''

import rospy
from geometry_msgs.msg import Twist
from orunav_msgs.msg import ControllerTrajectoryChunk,ControllerTrajectoryChunkVec, ControllerTrajectoryStep, ControllerCommand, ControllerReport, ControllerState


class controller2():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params        
        self.last_command = 1 #brake
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [joyController] STARTED")
    
        rospy.spin()
    
    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4) 

        self.reports_topic = rospy.get_param('~reports_topic', '/robot4/control/controller/reports') 

        self.command_topic = rospy.get_param('~command_topic', '/robot4/control/controller/commands') 
 
        self.twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 

        self.trajectory_topic = rospy.get_param('~trajectory_topic', '/robot4/control/controller/trajectories')

    def initROS(self):
        self.traj_pub = rospy.Publisher(self.trajectory_topic, ControllerTrajectoryChunkVec, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.command_topic, ControllerCommand, queue_size=1)
    
        rospy.Subscriber(self.twist_cmd_topic, Twist, self.twist_callback, queue_size=1)
        rospy.Subscriber(self.reports_topic, ControllerReport, self.reports_callback, queue_size=1)

        

    def reports_callback(self,msg):
        self.state=msg.state             

    def twist_callback(self,msg):
        # can't publish without state
        if hasattr(self, 'state'):
            command = ControllerCommand()

                # tell robot to use it!
            if self.last_command ==1: #brake
                command.robot_id = self.robot_id
                command.traj_id = 0
                
                command.command = 2
                self.last_command = 2
                self.cmd_pub.publish(command)
            
                command.command = 3
                self.last_command = 3
                command.start_time = rospy.Time.now() 
                self.cmd_pub.publish(command)
                print('z')

            # what I get from the incoming data
            v     = msg.linear.x
            phi = msg.angular.z

            if v == phi == 0.0:
                command.command = 1
                self.last_command = 1
                command.start_time = rospy.Time.now() 
                self.cmd_pub.publish(command)

            ctcV = ControllerTrajectoryChunkVec()
            ctc = ControllerTrajectoryChunk()
            ctc.robot_id=self.robot_id
            ctc.traj_id = 0
            # only one chunk in the vec.
            ctc.sequence_num=0
            ctc.final=True

            # start
            start = ControllerTrajectoryStep()
            start.mode = ControllerTrajectoryStep.MODE_1

            # state we need it to achieve
            start.state =self.getNewState(self.state,v,phi)

            start.velocities.tangential=v
            start.velocities.steering=(phi-self.state.phi)/dt
            ctc.steps.append(start)

            ctcV.chunks.append(ctc)

            self.traj_pub.publish(ctcV)

        else:
            print('.')

def getNewState(self,curr_state,vm,phi):
    newState = State()
    



    return newState

# Main function.
if __name__ == '__main__':
        rospy.init_node('twistController')#, log_level=rospy.DEBUG)
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=controller2()
        except rospy.ROSInterruptException:
            pass

