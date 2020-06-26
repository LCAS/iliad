#!/usr/bin/env python


'''

Plots QTCc interactions in an rviz marker
Visuals are updated periodically

'''
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from projector_interface.srv import DrawPolygon, DrawPolygonRequest, DrawPolygonResponse, ClearPolygons
from geometry_msgs.msg import Point32
from std_msgs.msg import ColorRGBA

WHITE = ColorRGBA (255,255,255,  0)
BLACK = ColorRGBA (  0,  0,  0,  0)
RED   = ColorRGBA (255,  0,  0,  0)
GREEN = ColorRGBA (  0,255,  0,  0)
BLUE  = ColorRGBA (  0,  0,255,  0)


class QTCStatePlotterNode():


    
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes      
        # sensible default values ...
        # In camera coordinates 0,0 is just in front of the camera and increases down (y) right(x)
        #
         
        self.x0 = -2.33 + 5.2 #-2.33
        self.y0 = 0 #-1.91
        self.z0 = 0.01
        self.x_size = 0.1#5.2
        self.y_size = 0.1#2.7

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] entering spin...")

        r = rospy.Rate(self.visuals_publish_rate)
        while not rospy.is_shutdown():
            self.updateVisuals()
            r.sleep()

        # final cleaning call ...
        try:
            self.clear_polig_srv_px.call()
        except rospy.ServiceException as e:
            pass
        

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        # current robot frame ide from tf tree
        self.robot_base_frame_id = rospy.get_param('~robot_base_frame_id', '/camera') #'/robot' + str(self.robot_id) + '/base_link')

        # poligon draw service
        self.draw_polig_srv_name = rospy.get_param('~draw_polig_srv_name', '/draw_polygon')
        self.clear_polig_srv_name = rospy.get_param('~clear_polig_srv_name', '/clear_polygons')

        # how ofen we publish
        self.visuals_publish_rate = rospy.get_param('~visuals_publish_rate', 1.0)

    def initROS(self):
        # publishers
        # none

        # service clients
        
        rospy.loginfo("Node [" + rospy.get_name() + "] waiting for service [" +self.draw_polig_srv_name +"] to be available")
        rospy.wait_for_service(self.draw_polig_srv_name)
        try:
            self.draw_polig_srv_px = rospy.ServiceProxy(self.draw_polig_srv_name, DrawPolygon)
        except rospy.ServiceException as e:
            rospy.logerr("Node [" + rospy.get_name() + "] Could not create srv proxy: " + str(e))

        rospy.loginfo("Node [" + rospy.get_name() + "] waiting for service [" +self.clear_polig_srv_name +"] to be available")
        rospy.wait_for_service(self.clear_polig_srv_name)
        try:
            self.clear_polig_srv_px = rospy.ServiceProxy(self.clear_polig_srv_name, ClearPolygons)
        except rospy.ServiceException as e:
            rospy.logerr("Node [" + rospy.get_name() + "] Could not create srv proxy: " + str(e))
        # transform buffers

        # subscribers and listeners

        # service servers
        # none here

    def updateVisuals(self):
        data = DrawPolygonRequest()
        data.label = 'hola caracola'
        data.polygon.header.frame_id = self.robot_base_frame_id

        rospy.loginfo("Node [" + rospy.get_name() + "] Printing at: ("+ str(self.x0) + ", " + str(self.y0) +")")  

        point = Point32(self.x0,self.y0, self.z0)
        data.polygon.polygon.points.append(point)
        point = Point32(self.x0,self.y0+self.y_size, self.z0)
        data.polygon.polygon.points.append(point)
        point = Point32(self.x0+self.x_size,self.y0+self.y_size, self.z0)
        data.polygon.polygon.points.append(point)
        point = Point32(self.x0+self.x_size,self.y0, self.z0)
        data.polygon.polygon.points.append(point)
                
        point = Point32(self.x0,self.y0, self.z0)
        data.text_rect.points.append(point)
        point = Point32(self.x0,self.y0+self.y_size, self.z0)
        data.text_rect.points.append(point)
        point = Point32(self.x0+self.x_size,self.y0+self.y_size, self.z0)
        data.text_rect.points.append(point)
        point = Point32(self.x0+self.x_size,self.y0, self.z0)
        data.text_rect.points.append(point)

        data.color = RED

        #self.x0 = (self.x0 + 3.0 + 0.1) % (6.0) -3.0

        # Finally call service .......................
        try:
            self.clear_polig_srv_px.call()
            self.draw_polig_srv_px.call(data)
        except rospy.ServiceException as e:
            rospy.logerr("Node [" + rospy.get_name() + "] Could not call service: [" + self.draw_polig_srv_name + "] to Abort task:" + str(e))

# Main function.
if __name__ == '__main__':
    rospy.init_node('QTCStatePlotterNode') # log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = QTCStatePlotterNode()
    except rospy.ROSInterruptException:
        pass
