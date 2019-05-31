#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <orunav_msgs/SetTask.h>

#include <dynamic_constraints/constraintSettingsConfig.h>

class DynamicConstraintsNode {

private:
  ros::ServiceClient service_client_;
  ros::ServiceServer service_server_;
  bool use_ct_;

  orunav_msgs::Task task_;
  
 public:
  DynamicConstraintsNode(ros::NodeHandle &nh)
  {
      // Parameters
      // TODO this param should be loaded from the coordinator_fake_node
      nh.param<bool>("use_ct", use_ct_, true);
      
      // Service
      service_server_ = nh.advertiseService("execute_task_coordinator", &DynamicConstraintsNode::setTaskCB, this);
      // Client
      service_client_ = nh.serviceClient<orunav_msgs::SetTask>("execute_task");


      dynamic_reconfigure::Server<dynamic_constraints::constraintSettingsConfig> dyn_srv;
      dynamic_reconfigure::Server<dynamic_constraints::constraintSettingsConfig>::CallbackType f;
      f = boost::bind(&DynamicConstraintsNode::dynamic_reconfigure_callback,this, _1, _2);
      dyn_srv.setCallback(f);


    }


void dynamic_reconfigure_callback(dynamic_constraints::constraintSettingsConfig &config, uint32_t level)
{
  //ROS_INFO("Reconfigure request : %f",
  //         config.speed_factor);

  // TODO do nothing for now
}
  


  // TODO this method is the same than the one in coordinator_fake_node.cpp
  bool validTaskMsg(const orunav_msgs::Task &task) const {
    // Any path points?
    if (task.path.path.size() < 3) {
      ROS_ERROR_STREAM("[DynamicConstraintsNode] Not a valid task: path to short, current length : " << task.path.path.size()); 
      return false;
    }
    if (use_ct_) {
      if (task.dts.dts.size() < 2) { // 2- the fastest and slowest
        ROS_ERROR_STREAM("[DynamicConstraintsNode] Not a valid task: no dts vectors, current length: " << task.dts.dts.size());
        return false;
      }
      if (task.path.path.size() != task.dts.dts[0].dt.size()) {
        ROS_ERROR_STREAM("[DynamicConstraintsNode] Not a valid task: dts[0] length different from path length: " << task.dts.dts[0].dt.size());
        return false;
      }
    }
    if (task.path.path.size() < task.constraints.constraints.size()) {
      ROS_ERROR_STREAM("[DynamicConstraintsNode] Not a valid task: amount of constraints larger the path length: " << task.constraints.constraints.size());
      return false;
    }
    return true;
  }
  
  // Service callbacks
  bool setTaskCB(orunav_msgs::SetTask::Request &req,
                 orunav_msgs::SetTask::Response &res)
  {
    orunav_msgs::Task inTask = req.task;
    orunav_msgs::SetTask outSetTask;
    
    ROS_INFO("[DynamicConstraintsNode] RID:%d - received setTask", (int) inTask.target.robot_id);

    // Verify the received task
    

    if (!validTaskMsg(inTask)) {
      ROS_ERROR("[DynamicConstraintsNode] received invalid task.");
      return false;
    }

    // HERE WE SHOULD DO OUR STUFF modifying inTask
    

    // Once modified, forward the task to the execution node
    
    outSetTask.request.task = inTask;
              
    if (service_client_.call(outSetTask)) {
      ROS_INFO("[DynamicConstraintsNode] - set_task sucessfull");
    }else{
      ROS_ERROR("[DynamicConstraintsNode] - Failed to call service: set_task");
      return false;
    }
      ROS_INFO_STREAM("[DynamicConstraintsNode] - set_task return value : " << outSetTask.response.result);

    // and finally propagate back the response
    res = outSetTask.response;
    return true;
  }

 




};


int main(int argc, char** argv) {

    ros::init(argc,argv,"dynamic_constraints");
    ros::NodeHandle nh;

    DynamicConstraintsNode cf(nh);

    ros::spin();

}
