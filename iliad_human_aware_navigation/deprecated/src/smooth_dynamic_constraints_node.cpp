#include <ros/ros.h>

/*

UNFINISHED. WE ARE NOW USING A DIFFERENT APPROACH

Subscribes to the following from VEN
  - Trajectory:
    Updates its pool of trajectories
    Triggers

  - Commands
    Sets as active the

Subscribes to the following from mpc
  - Reports:
    Updates its pointer to active chunk

Subscribes to the following topic
  - Constraints:
    Recomputes remaining trajectory chunks with new speed profile.
    Publishes active (and updated) trajectory into mpc

Publishes the following topics
  - Trajectory (to MPC):
    Trajectory with updated constraints


*/


class DynamicConstraintsNode {

private:
    int robot_id;
    ros::NodeHandle nodeHandle_;
    double timeStep;
    // trajectory chunks
    std::string ven_trajectories_topic_name;
    ros::CallbackQueue traj_queue;
    ros::Subscriber traj_subscriber;
    VehicleState vehicle_state_;

 public:
  DynamicConstraintsNode(ros::NodeHandle &nh) : nodeHandle_(nh)
  {
    // load ros params
    loadROSParams();

    // create ros connections
    createROSConnections();

    //loop
    ros::spin();

  }

  void loadROSParams(){
       ros::NodeHandle private_node_handle("~");

       // LOAD ROS PARAMETERS ....................................
       std::string temp;

       // robot id
       private_node_handle.param("robot_id", temp,std::string("4"));
       robot_id=std::stoi(temp);

       // ven_trajectories_topic_name
       temp = string("/robot") + str(robot_id) + string("/control/controller/bounds_tangential_velocity");
       private_node_handle.param("ven_trajectories_topic_name", ven_trajectories_topic_name, temp);

       // TODO READ THIS FROM VEHICLE EXECUTION NODE!!!
       private_node_handle.param<double>("time_step", timeStep, 0.06);

       vehicle_state_.setTimeStep(timeStep);

       // if (boost::iequals(temp, std::string("true"))) {
       //     loadGrids=true;
       // } else {
       //     loadGrids=false;
       // }

   }

   void createROSConnections(){


     ros::SubscribeOptions traj_sub_opts = ros::SubscribeOptions::create<orunav_msgs::ControllerTrajectoryChunkVec>(
             ven_trajectories_topic_name,
             1000,
             boost::bind(&DynamicConstraintsNode::TrajectoryCallback, this, _1),
             ros::VoidPtr(),
             &traj_queue);

     traj_subscriber = nodeHandle_.subscribe(traj_sub_opts);



   }

   void trajectoryCallback(const orunav_msgs::ControllerTrajectoryChunkVecConstPtr& msg){
     // we should store the trajectory ...

   }


   void sendTrajectoryChunks(const std::pair<unsigned int, orunav_generic::TrajectoryChunks> &chunks_data) {

     vehicle_state_.appendTrajectoryChunks(chunks_data.first, chunks_data.second);

     // Make sure that the ones that are in vehicle state are the ones that are sent to the controller....
     orunav_generic::TrajectoryChunks chunks = vehicle_state_.getTrajectoryChunks();
     orunav_msgs::ControllerTrajectoryChunkVec c_vec;

     // Send them off
     for (unsigned int i = chunks_data.first; i < chunks.size(); i++) {
       orunav_msgs::ControllerTrajectoryChunk c = orunav_conversions::createControllerTrajectoryChunkFromTrajectoryInterface(chunks[i]);

       c.robot_id = robot_id_;
       c.traj_id = 0;
       c.sequence_num = i;
       c.final = false;
       if (i == chunks.size() - 1)
         c.final = true;
       c_vec.chunks.push_back(c);
     }

     trajectorychunk_pub_.publish(c_vec);

   }


   void constraints(){

       unsigned int path_idx;
       // idx when we can safely connect
       unsigned int chunk_idx = vehicle_state_.getCurrentTrajectoryChunkIdx() + chunk_idx_connect_offset_; // This is the earliest we can connect to.
       unsigned int chunk_idx_to_end_margin = 5; // After we connect we need some chunks to drive before the end.

       bool new_path_is_shorter = (path.sizePath() < vehicle_state_.getPath().sizePath());
       if (new_path_is_shorter) {
         chunk_idx_to_end_margin = 1;
   ROS_INFO_STREAM("[KMOVehicleExecution] - SIZE of paths: new:" << path.sizePath() << " old:" << vehicle_state_.getPath().sizePath() << " curretn IDX:" << vehicle_state_.getCurrentPathIdx());
       }
       if (new_path_is_shorter || vehicle_state_.isChunkIdxValid(chunk_idx+chunk_idx_to_end_margin)) {
         // Two options
         // 1) the new path start is the same as the prev path start
         // 2) the new path start is the same as the prev path goal
         // In case of 1), we're all fine to continue...
         // In case of 2), we need to connect the provided path with the current path.
         ROS_INFO_STREAM("[KMOVehicleExecution] - (+3) chunk_idx: valid : " << chunk_idx);
         double path_chunk_distance;
         vehicle_state_.updatePath(path);
         if (use_ct_) {
           ROS_INFO_STREAM("Adding cts: " << cts.size());
           ROS_INFO_STREAM("Path size : " << path.sizePath());
           vehicle_state_.setCoordinatedTimes(cts);
         }
         if (vehicle_state_.isDrivingSlowdown()) {
           ROS_INFO_STREAM("[KMOVehicleExecution] - will drive in slowdown mode - ignoring CTs");
           vehicle_state_.clearCoordinatedTimes();
           bool valid = false;
           chunks_data = computeTrajectoryChunksCASE2(vehicle_state_, traj_slowdown_params_, chunk_idx, path_idx, path_chunk_distance, valid, use_ct_);
           if (!valid) {
             continue;
           }
           // Special case - clear the CT's -> however, should check that the CTS are not slower than the slowdown...
         }
         else {
           bool valid;
           chunks_data = computeTrajectoryChunksCASE2(vehicle_state_, traj_params_, chunk_idx, path_idx, path_chunk_distance, valid, use_ct_);
           if (!valid) {
             continue;
           }
         }
         if (use_ct_ && use_ahead_brake_) {
           double ahead_time;
           if (cts.isAhead(path_idx, ros::Time::now().toSec(), ahead_time)) {
             ROS_WARN_STREAM("[VehicleExecutionNode] - is ahead : " << ahead_time);
             if (ahead_time > 3.) {
               sendBrakeCommand(false); // CTS
             }
             continue;
           }
         }
         ROS_INFO_STREAM("[VehicleExecutionNode] - distance between the connected path state and the chunk_idx used : " << path_chunk_distance);
         // Check the distance > threshold (could also be to check what control input is required to bring it from state chunk to state path and to check if this is reasonable rather then a simple distance check...).
         vehicle_state_.setCurrentPathIdx(path_idx);
       }
       else {
         // The vehicle is simply to close to the final goal...
         // Instead of stopping, simply retry until we get into another state.
         ROS_INFO_STREAM("[KMOVehicleExecutionNode] - too close to the goal, will try again");
         usleep(1000000);
         vehicle_state_.setResendTrajectory(true);
         continue;
       }


   }

   void reportCallback(const orunav_msgs::ControllerReportConstPtr &msg) {

     bool completed_target, recompute_new_trajectory;
     vehicle_state_.update(msg, completed_target, recompute_new_trajectory, use_forks_);
   }

};


int main(int argc, char** argv) {

    ros::init(argc,argv,"smooth_dynamic_constraints_node");
    ros::NodeHandle nh;

    DynamicConstraintsNode cf(nh);


  ros::requestShutdown();
  return 0;

}
