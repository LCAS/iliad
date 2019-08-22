#pragma once

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>    
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>


#include <pluginlib/class_loader.h>
#include <nav_core/base_local_planner.h>
     

// - messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


// ROS - OURS
#include <orunav_msgs/ControllerState.h>
#include <orunav_msgs/Task.h>
#include <orunav_msgs/PoseSteering.h>

using namespace std;
using namespace ros;

namespace iliad_local_planner {

/*!
 
 */
class local_planner
{
      
    public:

      /*!
       * Constructor.
       * @param nodeHandle the ROS node handle.
       */
      local_planner(ros::NodeHandle& nodeHandle);

      virtual ~local_planner();
      
      //! callback for robot state...
      void stateCallback(const orunav_msgs::ControllerState::ConstPtr& state_msg);

      //! callback for Tasks...
      void taskCallback(const orunav_msgs::Task::ConstPtr& task_msg);

      void loadROSParams();
      
      void showROSParams();
      
      void initROSComms();

      std::vector<geometry_msgs::PoseStamped> task2path(orunav_msgs::Task inTask);
    private:
    



      boost::shared_ptr<nav_core::BaseLocalPlanner> local_planner_ptr_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> local_planner_loader_;
      costmap_2d::Costmap2DROS* local_costmap_ptr_;
      std::string local_planner_class_name_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      ros::Timer updateTimer_;
      double controller_frequency_;
      void updateLocalPathCallback(const ros::TimerEvent&);
      ros::Subscriber current_local_plan_sub_;
      std::string current_local_plan_topic_name_;
      void localPlanCallback(const nav_msgs::Path::ConstPtr& plan_msg);

      int robot_id_;

      //! ROS nodehandle.
      ros::NodeHandle& nodeHandle_;

      //! Robot State. It implicitly uses global_frame_id
      orunav_msgs::ControllerState robot_state_;
    
      //! ROS subscriber to robot state..
      ros::Subscriber robot_state_sub_;

      //! Robot state topic for the subscriber
      std::string robot_state_topic_name_;

      //! Robot Current task (from coordinator).
      orunav_msgs::Task current_task_;

      //! ROS subscriber to Tasks..
      ros::Subscriber current_task_sub_;

      //! Current Task topic for the subscriber
      std::string current_task_topic_name_;
    
      //tf stuff      
      tf::TransformListener listener_;
      tf::StampedTransform transform_;
      
      //! global frame id (world?). Is the one "implictly" used in coordinator
      std::string global_frame_;

      //! robot frame id (robotX/odom?)
      std::string robot_frame_;

}; // End of Class local_planner

} // end of  namespace iliad_local_planner
