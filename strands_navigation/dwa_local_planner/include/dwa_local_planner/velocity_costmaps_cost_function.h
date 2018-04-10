#ifndef VELOCITY_COSTMAPS_COST_FUNCTION_H_
#define VELOCITY_COSTMAPS_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/thread/mutex.hpp>

namespace dwa_local_planner {

/**
 * class ObstacleCostFunction
 * @brief Uses costmap 2d to assign negative costs if robot footprint
 * is in obstacle on any point of the trajectory.
 */
class VelocityCostmapsCostFunction : public base_local_planner::TrajectoryCostFunction {

public:
  VelocityCostmapsCostFunction();
  ~VelocityCostmapsCostFunction();

  bool prepare();
  double scoreTrajectory(base_local_planner::Trajectory &traj);

  void setParams(double max_vel_x) {
      max_vel_x_ = max_vel_x;
  }

  virtual float getCost(unsigned int cx, unsigned int cy){ return 0; }

  void callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

private:
  inline unsigned int getIndex(unsigned int mx, unsigned int my, unsigned int size_x) const
  {
    return my * size_x + mx;
  }

  inline void check_map() {
      if(ros::Time::now() > map_.header.stamp + ros::Duration(2))
          map_.info.width = 0;
  }

  double max_vel_x_, costs_xv_, costs_tv_;
  ros::Subscriber sub;
  ros::Publisher pub;
  nav_msgs::OccupancyGrid map_, pub_map_;
  boost::mutex mutex;
};

} /* namespace base_local_planner */
#endif /* VELOCITY_COSTMAPS_COST_FUNCTION_H_ */
