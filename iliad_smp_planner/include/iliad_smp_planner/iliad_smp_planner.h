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
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef iliad_smp_PLANNER_H_
#define iliad_smp_PLANNER_H_


#include <ros/ros.h>

#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/OccupancyGrid.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


#include <orunav_msgs/GetPath.h>

#include <orunav_generic/path_utils.h>
#include <orunav_generic/io.h>
#include <orunav_generic/utils.h>

#include <boost/thread/mutex.hpp>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace iliad_smp_planner{
  /**
   * @class iliad_smp_planner
   * @brief Provides a simple global planner (based on carrot-planner) 
   * that will call to iliad_smp service to retrieve paths.
   */
  class iliad_smp_planner : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the iliad_smp_planner
       */
      iliad_smp_planner();

      /**
       * @brief  Constructor for the iliad_smp_planner
       * @param  name The name of this planner
       */
      iliad_smp_planner(std::string name);

      /**
       * @brief  Initialization function for the iliad_smp_planner
       * @param  name The name of this planner
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);


      bool callSMP( orunav_msgs::RobotTarget target, nav_msgs::OccupancyGrid map, orunav_msgs::Path& path);

      orunav_msgs::RobotTarget castPoseStampedToOru(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal);

      bool getCurrentMap (nav_msgs::OccupancyGrid& map );

      bool castPath(geometry_msgs::PoseStamped start, orunav_msgs::Path path, std::vector<geometry_msgs::PoseStamped>& plan);

      void process_map(const nav_msgs::OccupancyGrid::ConstPtr &msg);


      bool castPoseToMapFrame(const geometry_msgs::PoseStamped &inPose, geometry_msgs::PoseStamped &outPose, std::string &map_frame_id);


    private:
      ros::ServiceClient iliad_smp_srv_client_;
      bool initialized_;
      
      nav_msgs::OccupancyGrid current_map_;
      boost::mutex map_mutex_;
      ros::Subscriber map_sub_;
      int task_id_inc_;
      int my_robot_id_;
      bool load_operation_;
      bool load_detect_;
      bool valid_map_;
      std::string service_name_;

      tf2_ros::Buffer tfBuffer_;
      tf2_ros::TransformListener tfListener_;

  };
};  
#endif




