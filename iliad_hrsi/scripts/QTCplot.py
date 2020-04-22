#!/usr/bin/env python


'''

Plots QTCc interactions in an rviz marker
Visuals are updated periodically

'''
import rospy
import tf2_geometry_msgs
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Float64MultiArray

from visualization_msgs.msg import MarkerArray, Marker



class QTCStatePlotterNode():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes      
        # sensible default values ...
        self.qtc_data = None
        self.qtc_state = None
        self.arrow_len = 1.2

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] entering spin...")

        r = rospy.Rate(self.visuals_publish_rate)
        while not rospy.is_shutdown():
            self.updateVisuals()
            r.sleep()

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        # Current QTC comes from here
        self.qtc_state_topic = rospy.get_param('~qtc_state_topic', '/robot'+str(self.robot_id)+'/qsr/qtc_state')

        # current robot frame ide from tf tree
        self.robot_base_frame_id = rospy.get_param('~robot_base_frame_id', '/robot' + str(self.robot_id) + '/base_link')

        # points used in qtc is also published
        self.qtc_points_topic_name = rospy.get_param('~qtc_points_topic_name', '/robot' + str(self.robot_id) + '/qsr/qtc_points')

        # MARKER for visual data
        self.visual_topic = rospy.get_param('~visual_topic', '/robot'+str(self.robot_id)+'/qsr/visualization_marker')

        # global frame id: this is where we publish the visuals
        self.global_frame_id = rospy.get_param('~global_frame', 'map_laser2d')
        # in tf2, frames do not have the initial slash
        if (self.global_frame_id[0] == '/'):
            self.global_frame_id = self.global_frame_id[1:]
        
        # tranform tf_timeout
        timeout = rospy.get_param('~tf_timeout', 2)
        self.tf_timeout = rospy.Duration(timeout)

        # how ofen we publish
        self.visuals_publish_rate = rospy.get_param('~visuals_publish_rate', 2.0)

        # show qtc or text?
        self.use_pretty_text = rospy.get_param('~pretty_text', True) 


    def initROS(self):
        # publishers
        self.visual_pub = rospy.Publisher(self.visual_topic, MarkerArray, queue_size=1)

        # service clients
        # none here

        # transform buffers
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # subscribers and listeners
        rospy.Subscriber(self.qtc_state_topic, String, self.qtc_state_callback, queue_size=1)
        rospy.Subscriber(self.qtc_points_topic_name, Float64MultiArray, self.qtc_points_callback, queue_size=1)        


        # service servers
        # none here


    def qtc_points_callback(self, msg):
        self.qtc_data = msg.data
     
    def qtc_state_callback(self, msg):
        self.qtc_state = msg

    def transformPoseSt(self, poseSt_in, frame_id_out):
        """Transform pose into provided frame id."""
        poseSt_out = None
        now = rospy.Time.now()

        # in tf2, frames do not have the initial slash
        if (frame_id_out[0] == '/'):
            frame_id_out = frame_id_out[1:]
        # in tf2, frames do not have the initial slash
        if (poseSt_in.header.frame_id[0] == '/'):
            poseSt_in.header.frame_id = poseSt_in.header.frame_id[1:]

        try:
            transform = self.tfBuffer.lookup_transform(frame_id_out, poseSt_in.header.frame_id, now, self.tf_timeout)
            poseSt_out = tf2_geometry_msgs.do_transform_pose(poseSt_in, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("[%s] transform from (%s) to (%s) failed: (%s).", rospy.get_name(), poseSt_in.header.frame_id, frame_id_out, e)        
        
        return poseSt_out

    def getMarker(self, type, x0,y0,x1,y1,r,g,b,ns):
        line = Marker()
        line.id = 0
        line.type = type
        line.header.frame_id = self.global_frame_id

        line.header.stamp = rospy.Time.now()
        line.ns = ns
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0

        # LINE_STRIP markers use only the x component of scale, for the line width
        line.scale.x = 0.05

        # color
        line.color.r = r
        line.color.g = g
        line.color.b = b
        line.color.a = 1.0

        startP = Point(x0, y0, 1)
        
        endP = Point(x1, y1, 1)
        line.points.append(startP)
        line.points.append(endP)
        return line

    def updateVisuals(self):
            validData = self.qtc_data and self.qtc_state
            
            if validData:
                    (xh0, yh0, hh0, xh1, yh1, hh1, xr0, yr0, hr0, xr1, yr1, hr1) = self.qtc_data
                    tx = (xh0+xr0)/2.0
                    ty = (yh0+yr0)/2.0

                    data = MarkerArray()

                    #  connecting line (blue) .............................................. 
                    line = self.getMarker(Marker.LINE_STRIP,xh0,yh0,xr0,yr0,0,0,1,"connecting_line")
                    data.markers.append(line)

                    # robot motion (red) .......................................................
                    line = self.getMarker(Marker.ARROW, xr0,yr0,xr1,yr1,1,0,0,"robot_motion")                
                    data.markers.append(line)

                    # human motion (green) .......................................................                 
                    line = self.getMarker(Marker.ARROW, xh0,yh0,xh1,yh1,0,1,0,"human_motion")
                    data.markers.append(line)
                    # mfc should these lines be longer?

                    # descriptive text  .......................................................
                    text = Marker()
                    text.id = 1
                    text.type = Marker.TEXT_VIEW_FACING
                    text.header.frame_id = self.robot_base_frame_id
                    text.header.stamp = rospy.Time.now()
                    text.ns = "descriptor"
                    text.action = Marker.ADD
                    text.pose.orientation.w = 1.0

                    text.pose.position.x = tx
                    text.pose.position.y = ty
                    text.pose.position.z = 1+0.2
                    # TEXT_VIEW_FACING markers use only the z component of scale, specifies the height of an uppercase "A".
                    text.scale.z = 0.45

                    # we show next state as text
                    text.text = (self.qtc_state.data)
                    if (self.use_pretty_text):
                        text.text = self.getPrettyText(text.text)
                    # red color
                    text.color.r = 1 
                    text.color.g = 0.0
                    text.color.a = 1.0
                        
                    data.markers.append(text)

                    # Finally publish .......................
                    self.visual_pub.publish(data)

            rospy.logdebug_throttle(1, "Node [" + rospy.get_name() + "] " +
                                                "Updated visuals ... "
                                                )

    def getPrettyText(self, qtc_state):
        qtc_data = qtc_state.split(',')
        human_h = qtc_data[0]
        robot_h = qtc_data[1]
        human_v = qtc_data[2]        
        robot_v = qtc_data[3]

        text_h = 'H:' + self.getPhrase(human_h, human_v)

        text_r = 'R:' + self.getPhrase(robot_h, robot_v)

        text = text_h + '\n' + text_r
        rospy.logdebug_throttle(1, "Node [" + rospy.get_name() + "] " +
                                                text
                                                )
        return text

    def getPhrase(self, h,v):
        text = self.getPrettyTextDist(h) +  self.getPrettyTextAngle(v)
        if h == v  == '0':
            text = 'static'
        return text

    def getPrettyTextAngle(self, symbol):
        if symbol == '-':
                text = ' from left' 
        elif symbol == '0':
                text = ''
        elif symbol == '+':
                text = ' from right' 

        return text
        
    def getPrettyTextDist(self, symbol):
        if symbol == '-':
                text = 'approaching' 
        elif symbol == '0':
                text = 'keeping dist.'
        elif symbol == '+':
                text = 'moving away' 
        return text

# Main function.
if __name__ == '__main__':
    rospy.init_node('QTCStatePlotterNode') # log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = QTCStatePlotterNode()
    except rospy.ROSInterruptException:
        pass
