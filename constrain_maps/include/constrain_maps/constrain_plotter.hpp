/**
 *
 * */

#pragma once
#include <time.h>       /* time_t, time, ctime */
#include <math.h>
#include <list>
#include <cmath>
#include <string>

// ROS
#include <ros/ros.h>
#include <ros/console.h>

// - messages
#include <orunav_msgs/ControllerTrajectoryChunkVec.h>
#include <orunav_msgs/ControllerTrajectoryChunk.h>
#include <orunav_msgs/ControllerReport.h>
#include <orunav_msgs/ControllerConstraints.h>

// - grid map
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include "grid_map_core/iterators/PolygonIterator.hpp"
#include "grid_map_core/Polygon.hpp"


//
#include <mutex>


using namespace std;
using namespace ros;
using namespace grid_map;

namespace constrain_maps {

/*!

 */
class constrain_plotter
{

    public:

      /*!
       * Constructor.
       * @param nodeHandle the ROS node handle.
       */
      constrain_plotter(ros::NodeHandle& nodeHandle);

      virtual ~constrain_plotter();

      void loadROSParams();

      void showROSParams();

      //! callbacks for messages...
      void constrainCallback(const orunav_msgs::ControllerTrajectoryChunkVecConstPtr& msg);

      void reportsCallback(const orunav_msgs::ControllerReportConstPtr& msg);

      //! periodic map updates
      void updateMapCallback(const ros::TimerEvent&);

      void restartTimer();

      void applyConstraints();
      void publishMap();
      void printLayerCorners(std::string layer);

      double getOther(double a0, double a1, double b,double y);
      Polygon getPolygon(double a0, double a1, double b);
      bool obeysConstrain(double a0, double a1, double b,Position p);


    private:
      std::mutex g_lock;
      //! ROS nodehandle.
      ros::NodeHandle& nodeHandle_;

      //! ROS subscriber to traj messages...
      ros::Subscriber traj_sub_;
      // subscribe to report messages
      ros::Subscriber rep_sub_;
      ros::Timer timer_;
      //! chunks!
      std::string traj_topic_name;
      std::string report_topic_name;

      // last received data
      orunav_msgs::ControllerTrajectoryChunkVec last_traj_vec;
      int last_traj_num_chunks;
      orunav_msgs::ControllerReport last_report;
      int currentTrajectoryChunkIdx_; // From the controller reports.


      //! Grid map data.
      grid_map::GridMap map_;

      //! Grid map layer name
      std::string layerName;

      //! Grid map layer lowest value
      double lower_value;

      //! Grid map layer highest value
      double upper_value;

      //! map Size (in meters)
      double size_x;
      double size_y;
      //! map resolution pixel/meter
      double resolution;
      //! 2d position of the grid map in the grid map frame [m].
      double orig_x=0;
      double orig_y=0;

      //! Grid map publisher.
      ros::Publisher gridMapPublisher_;

      //! grid map publisher topic name
      std::string grid_map_topic_name;

      //! grid map frame
      std::string grid_map_frame_id;

      //! update map period [s]
      double map_update_period;



}; // End of Class constrain_plotter

} // end of  namespace constrain_maps
