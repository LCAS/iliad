#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Periodically calls service orunav_msgs/ComputeTask  at  /robotX/compute_task 
to have an alternative task stored.

Upon request on empty type topic:
Calls service orunav_msgs/UpdateTask  at  /coordinator/update_task 
to update task with alternative task.
"""


import rospy
from orunav_msgs.srv import  UpdateTask, UpdateTaskRequest, UpdateTaskResponse
from orunav_msgs.srv import  ComputeTask, ComputeTaskRequest, ComputeTaskResponse
from orunav_msgs.msg import Operation
from orunav_msgs.msg import Task
from orunav_msgs.msg import ControllerReport
from std_msgs.msg import Empty
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from scipy.interpolate import UnivariateSpline

class TaskReplanner():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes
        self.state = None
        self.active_task = None
        self.computed_task = None

        # MFC: find out reasonable values
        self.small_v = 0.01
        self.small_w = np.pi/10
        self.normal_v = 2
        self.normal_w = np.pi/2

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] Started.")



    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)
        self.wheelbase = rospy.get_param('~wheelbase', 1.2)

        self.update_task_srv_name = rospy.get_param(
            '~update_task_srv_name', '/coordinator/update_task')

        self.compute_task_srv_name = rospy.get_param(
            '~compute_task_srv_name', '/robot' + str(self.robot_id) + '/compute_task')

        self.curr_task_topic_name  = rospy.get_param(
            '~task_topic_name', '/robot' + str(self.robot_id) + '/control/task')

        self.trigger_topic_name  = rospy.get_param(
            '~trigger_topic_name', '/robot' + str(self.robot_id) + '/update_task')

        self.reports_topic_name = rospy.get_param(
            '~reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports')

        self.vel_constraints_topic_name = rospy.get_param(
            '~vel_constraints_topic', '/robot'+str(self.robot_id)+'/velocity_constraints')

        self.brake_topic_name = rospy.get_param(
            '~brake_topic_name', '/robot'+str(self.robot_id)+'/brake')            

        #self.plan_update_period = rospy.get_param('~plan_update_period', 2)

    def initROS(self):
        # topic publishers
        self.vel_constraints_topic_pub = rospy.Publisher(self.vel_constraints_topic_name, Float64MultiArray, queue_size=1)
        self.brake_topic_pub = rospy.Publisher(self.brake_topic_name, Int32, queue_size=1)
        
        # service clients
        rospy.loginfo("Node [" + rospy.get_name() + "] waiting for service [" +self.update_task_srv_name +"] to be available")
        rospy.wait_for_service(self.update_task_srv_name)
        try:
            self.update_task_srv_px = rospy.ServiceProxy(self.update_task_srv_name, UpdateTask)
        except rospy.ServiceException as e:
            rospy.logerr("Node [" + rospy.get_name() + "] Could not create srv proxy: " + str(e))

        # .......................
        rospy.loginfo("Node [" + rospy.get_name() + "] waiting for service [" +self.compute_task_srv_name +"] to be available")
        rospy.wait_for_service(self.compute_task_srv_name)
        try:
            self.compute_task_srv_px = rospy.ServiceProxy(self.compute_task_srv_name, ComputeTask)
        except rospy.ServiceException as e:
            rospy.logerr("Node [" + rospy.get_name() + "] Could not create srv proxy: " + str(e))

        # service servers
        
        # topic subscribers and other listeners
        rospy.Subscriber(self.curr_task_topic_name, Task, self.curr_task_callback, queue_size=1)
        rospy.Subscriber(self.reports_topic_name, ControllerReport, self.reports_callback, queue_size=1)
        rospy.Subscriber(self.trigger_topic_name, Empty, self.trigger_callback, queue_size=1)

        # Timers
        #rospy.Timer(rospy.Duration(self.plan_update_period), self.update_plan, oneshot=False)

    def curr_task_callback(self,msg):
        self.active_task = msg
        rospy.loginfo("Node [" + rospy.get_name() + "] Detected a new active Task!")

    def trigger_callback(self,msg):
        rospy.loginfo("Node [" + rospy.get_name() + "] Trying to replan it!")
        self.replan_task()    

    def publishSpeeds(self,constraint_v,constraint_w):
        msg = Float64MultiArray()
        msg.data.append(constraint_v)
        msg.data.append(constraint_w)
        self.vel_constraints_topic_pub.publish(msg)

    def stop(self,isStopping):
        msg = Int32()
        msg.data = 2 # RECOVER
        if isStopping:
            msg.data = 0 # STOP
        self.brake_topic_pub.publish(msg)

    def slowDown(self,isSlowDown):
        msg = Int32()
        msg.data = 2 # RECOVER
        if isSlowDown:
            msg.data = 1 #SLOWDOWN
        self.brake_topic_pub.publish(msg)        

    #def update_plan(self, event):
    def update_plan(self, start,goal):        
        if ((not self.state == None) and (not self.active_task == None)):
            req = ComputeTaskRequest()

            
            req.target = self.active_task.target
            
            if (start == None):
                req.start_from_current_state = True
            else:
                req.target.start = start
                req.start_from_current_state = False

            req.target.goal = goal
            try:
                ans = self.compute_task_srv_px.call(req)
                if ans.result == 0:
                    rospy.logerr("Node [" + rospy.get_name() + "] Compute task result was FAILED: "+str(ans.result))   
                    self.computed_task = None
                else:
                    rospy.loginfo("Node [" + rospy.get_name() + "] Compute task result was SUCCESS: "+str(ans.result))   
                    self.computed_task = ans.task                
            except rospy.ServiceException as e:
                rospy.logerr("Node [" + rospy.get_name() + "] Could not call service: [" + self.compute_task_srv_name + "]:" + str(e))
                self.computed_task = None

            
    # we use robot reports to know robot position 
    def reports_callback(self, msg):
            self.state = msg.state

    def findClosestInPath(self,path):
        # Not the smartest search ...
        dist = 1e10
        ind = -1
        for i,posSteer in enumerate(path):
            posit_i = posSteer.pose.position
            dist_i = self.getDist(posit_i,self.state)
            if (dist>dist_i):
                ind = i
                dist = dist_i
        return ind

    def getDist(self, position_i, state_i ):
        # I really don't like substracting points assuming same refernce frame ...
        dx = state_i.position_x - position_i.x
        dy = state_i.position_y - position_i.y
        # dt = state_i.orientation_angle - 
        dist = np.sqrt(dx*dx + dy*dy)   
        return dist     
        
    def get_rotation (self,orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw
    
    def printPath(self, task_path,label):
        n_points = len(task_path)
        x = np.array([task_path[i].pose.position.x for i in range(0,n_points)] )
        y = np.array([task_path[i].pose.position.y for i in range(0,n_points)] )

        self.pythonPrint(x,y,label)

        for i in range(0,n_points):
                    rospy.loginfo("P[" + str(i)+  "]  == " '{0:.2f}'.format(x[i]) +', {0:.2f}'.format(y[i]) )
        rospy.loginfo("#xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        rospy.loginfo("\n")

    def pythonPrint(self, x,y,label):
        xstr = "x" + label + " = "+ np.array2string(x, precision=2, separator=', ', suppress_small=True) + " " 
        ystr = "y" + label + " = "+ np.array2string(y, precision=2, separator=', ', suppress_small=True) + " " 

        print("#xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        print("\n")
        print(xstr)
        print("\n")
        print(ystr)
        print("\n")
        print("#xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        print("\n")
    
    def resample(self,x,y,n_samples):
        # credit to xdze2 at https://stackoverflow.com/questions/   52014197/how-to-interpolate-a-2d-curve-in-python

        # Degree of the smoothing spline. Must be <= 5. Default is k=3, a cubic spline.
        spline_deg = 3
        # Positive smoothing factor used to choose the number of knots. 
        smooth = .2
        
        # pack input points 
        x = np.array(x).flatten()
        y = np.array(y).flatten()
        points = np.vstack( (x, y) ).T

        # Linear length along the line:
        distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )) )
        distance = np.insert(distance, 0, 0)/distance[-1]

        # Build a list of the spline function, one for each dimension:
        splines = [UnivariateSpline(distance, coords, k=spline_deg, s=smooth) for coords in points.T]

        # Computed the spline for the asked distances:
        alpha = np.linspace(0, 1, n_samples)
        points_fitted = np.vstack( [spl(alpha) for spl in splines] ).T

        new_x = points_fitted[:,0]
        new_y = points_fitted[:,1]
        return (new_x, new_y)
     
    def abort_task(self):        
        if ((not self.state == None) and (not self.active_task == None)):
            self.stop(True)
            rospy.sleep(0.35)
            
            req = UpdateTaskRequest()
            req.task = self.active_task
        
            # where are we?
            curr_index = self.findClosestInPath(self.active_task.path.path)
            index_offset = 4
            end_index = min(curr_index+index_offset,len(self.active_task.path.path)-1)
            req.task.path.path = req.task.path.path[:end_index]
            req.task.target.goal = req.task.path.path[-1]

            req.task.update = True
            req.task.abort = True

            self.stop(False)
            rospy.sleep(0.35)
            
            try:
                ans = self.update_task_srv_px.call(req)
                if ans.result != 0:
                    rospy.logerr("Node [" + rospy.get_name() + "] Abort task result was FAILED: "+str(ans.result))   
                else:
                    rospy.loginfo("Node [" + rospy.get_name() + "] Abort task result was SUCCESS: "+str(ans.result))   
            except rospy.ServiceException as e:
                rospy.logerr("Node [" + rospy.get_name() + "] Could not call service: [" + self.update_task_srv_name + "] to Abort task:" + str(e))


    def replan_task(self):        
        if ((not self.state == None) and (not self.active_task == None)):
            last_active_task = self.active_task    
            rospy.loginfo("Node [" + rospy.get_name() + "] Stopping to replan ......................")   
            self.abort_task()
            index_offset = 0
            old_points           = len(self.active_task.path.path)
            old_index_start      = self.findClosestInPath(self.active_task.path.path) + index_offset
            old_index_start      = min(old_points-1,old_index_start)
            old_remaining_points = old_points - old_index_start 

            start = None
            goal = self.active_task.target.goal
            rospy.loginfo("Node [" + rospy.get_name() + "] Getting new ......................")   
            self.update_plan(start, goal)
            
            if (self.computed_task != None):
                
                

                new_points           = len(self.computed_task.path.path)
                new_index_start      = self.findClosestInPath(self.computed_task.path.path)
                new_remaining_points = new_points - new_index_start 

                # rospy.loginfo("Node [" + rospy.get_name() + "] Computed Task:")   
                # rospy.loginfo("Node [" + rospy.get_name() + "] - In total has  " + str(new_points) + " points")               
                # rospy.loginfo("Node [" + rospy.get_name() + "] - Robot is at   " + str(new_index_start) + " point")   
                # rospy.loginfo("Node [" + rospy.get_name() + "] - Still to exec " + str(new_remaining_points) + " remaining points")   


                # rospy.loginfo("Node [" + rospy.get_name() + "] Active Task:")   
                # rospy.loginfo("Node [" + rospy.get_name() + "] - In total has  " + str(old_points) + " points")               
                # rospy.loginfo("Node [" + rospy.get_name() + "] - Robot is at   " + str(old_index_start) + " point")   
                # rospy.loginfo("Node [" + rospy.get_name() + "] - Still to exec " + str(old_remaining_points) + " remaining points")   

                # rospy.loginfo("Node [" + rospy.get_name() + "] We will change " + str(remaining_points) + " points")               
                # rospy.loginfo("Node [" + rospy.get_name() + "] Computed Task [" + str(new_index_start) + ", " + str(new_index_start+remaining_points-1) + 
                #                                             "] == Active Task [" + str(old_index_start) + ", " + str(old_index_start+remaining_points-1) + 
                #                                             "]")   
                
                if (old_remaining_points>1):
                    req = UpdateTaskRequest()
                    req.task = last_active_task
                
                    # So, we need to resample old_remaining_points from the vector computed_task_path[new_index_start:] and
                    # store them in active_task_path[old_index_start:]
                    # We assume both end in same point. 
                    x_pre = np.array([ path_i.pose.position.x for path_i in last_active_task.path.path[old_index_start:] ] )
                    y_pre = np.array([ path_i.pose.position.y for path_i in last_active_task.path.path[old_index_start:] ] )


                    x_new = np.array([ path_i.pose.position.x for path_i in self.computed_task.path.path[new_index_start:] ] )
                    y_new = np.array([ path_i.pose.position.y for path_i in self.computed_task.path.path[new_index_start:] ] )
                    (x_new_int, y_new_int) = self.resample(x_new, y_new, old_remaining_points)

                    steering = self.steeringAngles(x_new_int, y_new_int, self.wheelbase)
                    # self.pythonPrint(x_pre, y_pre, '_pre')
                    # self.pythonPrint(x_new, y_new, '_new')
                    # self.pythonPrint(x_new_int, y_new_int, '_new_int')

                    for i in range(0,old_remaining_points):
                        req.task.path.path[i + old_index_start].pose.position.x = x_new_int[i]
                        req.task.path.path[i + old_index_start].pose.position.y = y_new_int[i]
                        req.task.path.path[i + old_index_start].steering = steering[i]
                        

                    req.task.update = True
                    req.task.abort = False
                    
                    try:
                        ans = self.update_task_srv_px.call(req)
                        if ans.result != 0:
                            rospy.logerr("Node [" + rospy.get_name() + "] Update task result was FAILED: "+str(ans.result))   
                        else:
                            rospy.loginfo("Node [" + rospy.get_name() + "] Update task result was SUCCESS: "+str(ans.result))     
                    except rospy.ServiceException as e:
                        rospy.logerr("Node [" + rospy.get_name() + "] Could not call service: [" + self.update_task_srv_name + "]:" + str(e))


    def curvature(self, x, y):
          # credits to https://stackoverflow.com/questions/32629806/how-can-i-calculate-the-curvature-of-an-extracted-contour-by-opencv

          #init
          x_Old = x_Older = x[0]
          y_Old = y_Older = y[0]
          curvature = []
          for i in range(0, len(x)):   
            xi = x[i]
            yi = y[i]

            f1stD_x =   xi -       x_Old
            f1stD_y =   yi -       y_Old
            f2ndD_x = - xi + 2.0 * x_Old - x_Older
            f2ndD_y = - yi + 2.0 * y_Old - y_Older

            curvature2D = np.inf
            divisor = f2ndD_x + f2ndD_y

            if ( np.abs(divisor) > 10e-4 ):
                curvature2D = np.abs( f2ndD_y*f1stD_x - f2ndD_x*f1stD_y ) /  np.power( divisor, 1.5 )
            
            curvature.append(curvature2D)

            x_Older = x_Old
            y_Older = y_Old
            x_Old = xi
            y_Old = yi
          
          return curvature
    
    def steeringAngles(self, x, y, wheelbase):

        steerings = self.curvature(x,y)
        finite_curv = 0.0

        # get first finite one
        for i in range(0, len(x)):        
            if (np.isfinite(steerings[i])):
                finite_curv = steerings[i]
                break        
      
        steerings[0] = np.arctan(finite_curv * wheelbase)        

        for i in range(0, len(steerings)):       
            if (np.isfinite(steerings[i]) ):
                finite_curv = steerings[i]

        steerings[i] = np.arctan(finite_curv * wheelbase)

        return steerings

# Main function.
if __name__ == '__main__':
    rospy.init_node('task_replan_node', log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = TaskReplanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass






