/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include <iliad_smp_planner/iliad_smp_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(iliad_smp_planner::iliad_smp_planner, nav_core::BaseGlobalPlanner)

namespace iliad_smp_planner {

  iliad_smp_planner::iliad_smp_planner()
  :  initialized_(false),task_id_inc_(0){}

  iliad_smp_planner::iliad_smp_planner(std::string name)
  :  initialized_(false),task_id_inc_(0){
    initialize(name, NULL);
  }
  
  void iliad_smp_planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
      
    if(!initialized_){
      ros::NodeHandle private_nh("~/" + name);
      
      private_nh.param<bool>("load_operation",load_operation_,false);
      private_nh.param<bool>("load_detect",load_detect_,false);
      private_nh.param<int>("robot_id",my_robot_id_,1);

      iliad_smp_srv_client_ = private_nh.serviceClient<orunav_msgs::GetPath>("get_path");      
      map_sub_ = private_nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,&iliad_smp_planner::process_map, this);

      initialized_ = true;
    } else {
      ROS_WARN("This planner has already been initialized... doing nothing");
    }
  }

  bool iliad_smp_planner::callSMP(orunav_msgs::RobotTarget target, nav_msgs::OccupancyGrid map, orunav_msgs::Path& path){
      bool isOk =false; 
      orunav_msgs::GetPath srv;
       
      // TODO! This module totally overrides use_vector_map_and_geofence_ flag
      srv.request.map = map;      
      srv.request.target = target;
      srv.request.max_planning_time = 20.0; // TODO hardcoded param
      isOk = iliad_smp_srv_client_.call(srv);
              
      if (isOk) {
        ROS_DEBUG("[iliad_smp_planner] - get_path successful");
      } else {
        ROS_ERROR("[iliad_smp_planner] - Call to service get_path returns ERROR");               
      }
      
      // TODO This is another simplification of the real vehicle execution ...
      isOk = srv.response.valid;
      if (!isOk) {
        ROS_WARN("[iliad_smp_planner] no path found(!), cannot computeTask");        
      } else {
        path = srv.response.path;
      }
      
      return isOk;
  }
    
  orunav_msgs::RobotTarget iliad_smp_planner::castPoseStampedToOru(geometry_msgs::PoseStamped goal){
        // as seen in point_n_click_target_client.cpp
        
        orunav_msgs::RobotTarget target;
        
        target.goal.pose = goal.pose;
        target.goal.steering = 0.;
        target.goal_op.operation = target.goal_op.NO_OPERATION;
        target.start_op.operation = target.start_op.NO_OPERATION;

        if (load_operation_) {
            target.goal_load.status = target.goal_load.EUR_PALLET;        
            if ((task_id_inc_ % 2) == 0) {
                target.goal_op.operation = target.goal_op.LOAD;
                if (load_detect_) {
                    target.goal_op.operation = target.goal_op.LOAD_DETECT;
                }
                if (task_id_inc_ > 0) {
                    target.start_op.operation = target.start_op.UNLOAD;
                }
            }  else {
                target.goal_op.operation = target.goal_op.UNLOAD;
            }
        }

        target.robot_id = my_robot_id_;
        target.task_id = task_id_inc_++;  // This will anyway be handled by the coordinator.
        target.goal_id = target.task_id;  // Obsolete?
        target.start_earliest = ros::Time::now();

        return target;
    }  
  
  bool iliad_smp_planner::getCurrentMap (nav_msgs::OccupancyGrid& map ){

    if(!initialized_){
      ROS_ERROR("[iliad_smp_planner]  The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }  
    
    if(!valid_map_){
      ROS_ERROR("[iliad_smp_planner]  Map is not received");
      return false;
    }  
    
    map_mutex_.lock();
    map = current_map_;
    map_mutex_.unlock();
      
    return true;
  }

  bool iliad_smp_planner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    // more or less as seen in  orunav_vehicle_execution_node.cpp

    nav_msgs::OccupancyGrid map;
    bool isOk;
    orunav_msgs::RobotTarget oru_goal;
    orunav_msgs::Path oru_path;
    
    if(!initialized_){
      ROS_ERROR("[iliad_smp_planner]  The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    
    while (!getCurrentMap(map))
    { 
      ROS_ERROR("[iliad_smp_planner]  Map not ready... wait a second"); 
      usleep(1000);
    }
    
    // cast from move_base to oru target
    oru_goal = castPoseStampedToOru(goal);
    
    // ask iliad_smp for a path on the given map and path
    while (!callSMP(oru_goal,map,oru_path))
    { 
      ROS_ERROR("[iliad_smp_planner]  Can't get a path... wait a second"); 
      usleep(1000);
    }
    
    // cast oru path into ROS path
    isOk = castPath(start, oru_path,plan);
    
    return isOk;
  }
  
  bool iliad_smp_planner::castPath(geometry_msgs::PoseStamped start, orunav_msgs::Path path, std::vector<geometry_msgs::PoseStamped>& plan){
      bool isOk;
      isOk = true;
      
      // global planner plan is a list of PoseStamped
      
      // oru path contains the following:
      //     orunav_msgs/PoseSteering target_start
      //     orunav_msgs/PoseSteering target_goal
      //     orunav_msgs/PoseSteering[] path
      
      //        A PoseSteering is:
      //           geometry_msgs/Pose pose
      //           float64 steering
            
      for (unsigned int i = 0; i < path.path.size(); i++) {
        // quick way to copy header and such...
        geometry_msgs::PoseStamped new_goal = start;
        
        new_goal.pose = path.path[i].pose;
        plan.push_back(new_goal);
                                
      }
      
      return isOk;
  }

  void iliad_smp_planner::process_map(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    map_mutex_.lock();
    current_map_ = *msg;
    valid_map_ = true;
    map_mutex_.unlock();
  }

};
