#pragma once

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>    
#include <tf/transform_datatypes.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <ros/console.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/trajectory_planner_ros.h>
#include <pluginlib/class_loader.h>
#include <nav_core/base_local_planner.h>

// - messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

// ROS - ORU
#include <orunav_msgs/ControllerReport.h>
#include <orunav_msgs/RobotReport.h>
#include <orunav_msgs/Task.h>
#include <orunav_msgs/PoseSteering.h>
#include <orunav_msgs/ExecuteTask.h>

//others
#include <math.h>

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
      void robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

      //! callback for Tasks...
      void taskCallback(const orunav_msgs::Task::ConstPtr& task_msg);

      void loadROSParams();
      
      void showROSParams();
      
      void initROSComms();

      std::vector<geometry_msgs::PoseStamped> task2poseStVect(orunav_msgs::Task inTask);

    private:
    
      boost::shared_ptr<nav_core::BaseLocalPlanner> local_planner_ptr_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> local_planner_loader_;
      costmap_2d::Costmap2DROS* local_costmap_ptr_;
      std::string local_planner_class_name_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      
      int robot_status_;
      int controller_status_;
      ros::Subscriber robot_robot_report_sub_;
      std::string robot_robot_report_topic_name_;
      void robot_robot_report_callback(const orunav_msgs::RobotReport::ConstPtr& report_msg);

      ros::Timer updateTimer_;
      double controller_frequency_;
      void updateLocalPathCallback(const ros::TimerEvent&);
      ros::Subscriber current_local_plan_sub_;
      std::string current_local_plan_topic_name_;
      void localPlanCallback(const nav_msgs::Path::ConstPtr& plan_msg);
      void printROSParams();
      void printAllROSparams();
      int robot_id_;

      nav_msgs::Path poseStVect2Path(std::vector<geometry_msgs::PoseStamped> inPoseStVect);
      nav_msgs::Path global_path_;
      std::string teb_via_points_topic_name_;
      ros::Publisher teb_via_points_topic_name_pub_;
      
      // used to send ven the tasks.
      std::string ven_execute_task_srv_client_name_;
      ros::ServiceClient ven_execute_task_srv_client_;

      // used to get incoming tasks from coordinator
      std::string coord_execute_task_srv_name_;
      ros::ServiceServer coord_execute_task_srv_;
      bool coord_execute_task_callback(orunav_msgs::ExecuteTask::Request &req, orunav_msgs::ExecuteTask::Response &res);
      std::vector<orunav_msgs::PoseSteering> cast2PoseSteering(std::vector<geometry_msgs::PoseStamped> path_ros, double wheelbase); 

      //ros::Publisher temp_pub_;

      double getDist(geometry_msgs::PoseStamped poseStA, geometry_msgs::PoseStamped poseStB);
      nav_msgs::Path linear_subsample(nav_msgs::Path inputPath, unsigned int outLen);
      geometry_msgs::PoseStamped transformPose(std::string frame_id, geometry_msgs::PoseStamped poseIn);
      void printSuspicius(std::vector<geometry_msgs::PoseStamped> inV);

      void  robot_controller_reports_callback(const orunav_msgs::ControllerReport::ConstPtr& report_msg);

      std::vector<double> steeringAngles(std::vector<geometry_msgs::PoseStamped> path, double wheelbase);

      std::vector<double> curvature(std::vector<geometry_msgs::PoseStamped> path) ;

      double getYaw(geometry_msgs::Quaternion q0);
      void printPoseSteering(std::vector<orunav_msgs::PoseSteering> inV);
      void printPoseStamped(std::vector<geometry_msgs::PoseStamped> inV);

      //! ROS nodehandle.
      ros::NodeHandle& nodeHandle_;

      //! Robot position and speed, taken from odometry
      geometry_msgs::PoseStamped robot_poseSt_;
      geometry_msgs::TwistStamped robot_speedSt_;
    
      //! ROS subscriber to robot odometry (speed, pose)
      ros::Subscriber robot_odometry_sub_;

      //! Robot odometry topic name (robotX/odom by default)
      std::string robot_odometry_topic_name_;

      //! ROS subscriber to robot controller reports (we get state/steering_angle from it)
      ros::Subscriber robot_controller_reports_sub_;

      //! Robot controller reports topic name (robotX/control/controller/commands by default)
      std::string robot_controller_reports_topic_name_;

      double steering_;

      //! Robot Current task (from coordinator).
      orunav_msgs::Task current_task_;

      //! ROS subscriber to Tasks..
      ros::Subscriber current_task_sub_;

      //! Current Task topic for the subscriber
      std::string current_task_topic_name_;
    
      //tf stuff      
      tf::TransformListener listener_;
      
      // costamp needs a tf, but we will use tf2 for our transforms
      tf2_ros::Buffer tfBuffer2_;
      tf2_ros::TransformListener tf2_listener_;
      geometry_msgs::TransformStamped pose_to_frame_tf_; 
      ros::Time task_timestamp_;
      
      //! global frame id (world?). Is the one "implictly" used in coordinator
      std::string global_frame_;

      //! robot frame id: this is defined by whatever we get from the odom topic
      std::string robot_frame_id_;

}; // End of Class local_planner

} // end of  namespace iliad_local_planner
