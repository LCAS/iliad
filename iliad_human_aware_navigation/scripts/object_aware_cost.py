#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int64, Float64

from matplotlib import path
import numpy as np
import cell_tools as ctools
from threading import Lock

class checking_cells(object):
    def __init__(self):

        self.object_costamp_topic_name = rospy.get_param('~object_costamp_topic_name', '/object_costmap_node/costmap/costmap')
        self.envelope_topic_name = rospy.get_param('~envelope_topic_name', '/status')

        self.curr_cost_topic_name = rospy.get_param('~curr_cost_topic_name', '/object_aware/cost')
        self.envelope_gridmap_topic_name = rospy.get_param('~envelope_gridmap_topic_name', '/object_aware/envelope_gridmap')

        #init variables
        self.cells_points_computed = False
        self.envelope_received = False
        self.old_envelope_points = None
        self.lock = Lock()

        # init ros
        rospy.Subscriber(self.object_costamp_topic_name, OccupancyGrid, self.costmap_callback, queue_size=1)
        rospy.Subscriber(self.envelope_topic_name, MarkerArray,self.envelope_callback, queue_size=1)

        self.envelope_map_pub = rospy.Publisher(self.envelope_gridmap_topic_name, OccupancyGrid, queue_size=1)
        self.envelope_cost_pub = rospy.Publisher(self.curr_cost_topic_name, Float64, queue_size=1)

        self.update_obstacle_cost_timer = rospy.Timer(rospy.Duration(0.1), self.update_obstacle_cost,oneshot=False)
        rospy.spin()

    def costmap_callback(self, costmap_msg):
        if self.cells_points_computed ==False:
            self.cells_points_computed = True
            self.last_costmap = costmap_msg

            self.envelope_map_msg = OccupancyGrid()
            self.envelope_map_msg.header = costmap_msg.header
            self.envelope_map_msg.info = costmap_msg.info
            self.width = costmap_msg.info.width
            self.height = costmap_msg.info.height
            self.resolution =  costmap_msg.info.resolution
            self.envelope_map_msg.data = np.zeros(self.width*self.height)

            #create the x,y query points to pass to the inpolygon function
            self.xq = np.array([])
            self.yq = np.array([])
            x_chunk = np.array([])

            for row in range(0,self.width):
                x_chunk =np.append(x_chunk,(self.resolution/2)+row*self.resolution)

            for col in range(0,self.height):
                self.xq =np.append(self.xq,x_chunk)
                self.yq =np.append(self.yq,np.ones([1,self.width])*(self.resolution/2)+col*self.resolution)
        else:
            self.last_costmap = costmap_msg


    def envelope_callback(self,marker_msg):
        #print "robot status received"
        # check if the enveloped has changed
        envelope_points =  marker_msg.markers[2].points
        if envelope_points != self.old_envelope_points:
            #print "New envelope received..."
            self.old_envelope_points = envelope_points

            
            envelope_polygon_x = np.array([])
            envelope_polygon_y = np.array([])
            for pe in range(0, len(envelope_points)):
                envelope_polygon_x = np.append(envelope_polygon_x,envelope_points[pe].x)
                envelope_polygon_y = np.append(envelope_polygon_y,envelope_points[pe].y)

            #calculate the cells (using the center of the cells) inside the envelope
            envelope_map = np.zeros(self.width*self.height)
            envelope_cell_index = np.array([])
            points_inside = self.inpolygon(self.xq, self.yq, envelope_polygon_x , envelope_polygon_y )

            #set a value >0 to the cells inside the envelope
            for pi in range(0,len(points_inside)):
                if points_inside[pi] == True:
                    index = ctools.point2index(self.xq[pi],self.yq[pi],self.envelope_map_msg.info.origin.position.x,self.envelope_map_msg.info.origin.position.x+self.width*self.resolution,self.envelope_map_msg.info.origin.position.y,self.envelope_map_msg.info.origin.position.y+self.height*self.resolution,self.resolution,self.width, self.height)
                    envelope_cell_index = np.append(envelope_cell_index,index)
                    envelope_map[index] = 100            

            self.envelope_map_msg.data = envelope_map
            self.envelope_cell_index = envelope_cell_index
            self.envelope_received = True
            # publish an occupancy map to see in rviz the cells that are taken into account to compute the cost
            self.envelope_map_pub.publish(self.envelope_map_msg)
            rospy.loginfo("Enveloped updated")

    def update_obstacle_cost(self,timer):
        if self.envelope_received:
            costmap_data = self.last_costmap
            cell_index = self.envelope_cell_index
            cost = 0
            for i in range(0,len(cell_index)):
                cost = cost + costmap_data.data[int(cell_index[i])]

            if cost > 0:
                rospy.loginfo("cost: "+str(cost))
            self.envelope_cost_pub.publish(cost)


    # returns in indicating if the query points specified by xq and yq are inside 
    # of the polygon area defined by xv and yv.
    # https://stackoverflow.com/questions/31542843/inpolygon-for-python-examples-of-matplotlib-path-path-contains-points-method
    def inpolygon(self, xq, yq, xv, yv): 
        shape = xq.shape
        xq = xq.reshape(-1)
        yq = yq.reshape(-1)
        xv = xv.reshape(-1)
        yv = yv.reshape(-1)
        q = [(xq[i], yq[i]) for i in range(xq.shape[0])]
        p = path.Path([(xv[i], yv[i]) for i in range(xv.shape[0])])
        return p.contains_points(q).reshape(shape)



if __name__ == '__main__':
    rospy.init_node("task_cost_evaluator")#, log_level=rospy.DEBUG)
    try:
        cc = checking_cells()
    except rospy.ROSInterruptException:
        pass