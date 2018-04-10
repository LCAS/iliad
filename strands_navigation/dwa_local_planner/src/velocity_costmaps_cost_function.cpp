#include <dwa_local_planner/velocity_costmaps_cost_function.h>

#include <cmath>
#include <Eigen/Core>
#include <ros/console.h>

using base_local_planner::Trajectory;

namespace dwa_local_planner {

VelocityCostmapsCostFunction::VelocityCostmapsCostFunction() {
    costs_xv_ = 0.0;
    costs_tv_ = 0.0;

    ros::NodeHandle nh("~");
    std::string topic = "/velocity_costmap";
    nh.param("velocity_costmap_topic", topic, topic);
    sub = nh.subscribe(topic, 10, &VelocityCostmapsCostFunction::callback, this);
    pub = nh.advertise<nav_msgs::OccupancyGrid>("velocity_costmap", 10);

    //ROS_INFO("We're in namespace: %s", nh.getNamespace().c_str());
    
}

VelocityCostmapsCostFunction::~VelocityCostmapsCostFunction() {}


bool VelocityCostmapsCostFunction::prepare() {
    if(pub.getNumSubscribers() && map_.info.width != 0){
        pub.publish(pub_map_);
    }
    return true;
}

double VelocityCostmapsCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
//    ROS_INFO("VELMAPS");
    boost::mutex::scoped_lock lock(mutex);
    double cost = 0;
    if (sub.getNumPublishers() && map_.info.width != 0 && map_.info.height != 0) {
        double xv = traj.xv_ * 100;
        double tv = traj.thetav_ * M_PI;
        int x = xv * cos(tv);
        int y = xv * sin(tv);

        double xvt = max_vel_x_ * 100; // Projecting the angluar velocity outwards to prevent spinning on the spot when the velmap allows no movement.
        int xt = xvt * cos(tv);        // Might not be necessary, needs testing on robot.
        int yt = xvt * sin(tv);

        unsigned int index_x = VelocityCostmapsCostFunction::getIndex((x+(map_.info.width/2))-1, (y+(map_.info.height/2))-1, map_.info.width);
        unsigned int index_t = VelocityCostmapsCostFunction::getIndex((xt+(map_.info.width/2))-1, (yt+(map_.info.height/2))-1, map_.info.width);
        costs_xv_ = double(map_.data[index_x]);
        costs_tv_ = double(map_.data[index_t]);
//        cost = fmax(traj.xv_ * costs_xv_,fabs(traj.thetav_ * costs_tv_));
        cost = traj.xv_ * costs_xv_ + fabs(traj.thetav_ * costs_tv_);

//        ROS_INFO("Speed: x: %f, theta: %f, index: (%d, %d) = %d, %d -> costs: %.2f, x: %.2f, t: %.2f", traj.xv_, traj.thetav_, x, y, index_x, index_t, cost, traj.xv_ * costs_xv_, fabs(traj.thetav_ * costs_tv_));
        if(pub.getNumSubscribers()){
            pub_map_.data[index_x] = 98; // Just for visualisation, these costs have no effect on the planning
            pub_map_.data[index_t] = 50;
        }
        VelocityCostmapsCostFunction::check_map();
    }
    return cost;
}

void VelocityCostmapsCostFunction::callback(const nav_msgs::OccupancyGrid::ConstPtr &msg){
    boost::mutex::scoped_lock lock(mutex);
    map_ = *msg;
    pub_map_ = map_;
}
} /* namespace base_local_planner */
