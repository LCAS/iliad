#include <iliad_local_planner/iliad_local_planner.hpp>


namespace iliad_local_planner {
    
    
    local_planner::local_planner(ros::NodeHandle& n)
    : nodeHandle_(n),
    listener_(ros::Duration(10)),
    local_planner_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    local_costmap_ptr_(NULL),    
    global_frame_("world"),
    local_planner_class_name_("base_local_planner/TrajectoryPlannerROS")
    {

      loadROSParams();
      
      // Load plugins according to given ROS parameters

      local_costmap_ptr_= new costmap_2d::Costmap2DROS ("local_costmap", listener_);
      local_costmap_ptr_->pause();
      
      try {
          //check if a non fully qualified name has potentially been passed in
          if(!local_planner_loader_.isClassAvailable(local_planner_class_name_)){
            std::vector<std::string> classes = local_planner_loader_.getDeclaredClasses();
            for(unsigned int i = 0; i < classes.size(); ++i){
              if(local_planner_class_name_ == local_planner_loader_.getName(classes[i])){
                //if we've found a match... we'll get the fully qualified name and break out of the loop
                ROS_WARN("[%s]: Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                    ros::this_node::getName().c_str(), local_planner_class_name_.c_str(), classes[i].c_str());
                local_planner_class_name_ = classes[i];
                break;
              }
            }
          }
          //ROS_INFO("Fully quallified class name obtained");

          local_planner_ptr_ = local_planner_loader_.createInstance(local_planner_class_name_);
          //ROS_INFO("Created local_planner %s", local_planner_class_name_.c_str());

          local_planner_ptr_->initialize(local_planner_loader_.getName(local_planner_class_name_), &listener_, local_costmap_ptr_);
      } catch (const pluginlib::PluginlibException& ex)
      {
          ROS_FATAL("[%s]: Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", ros::this_node::getName().c_str(), local_planner_class_name_.c_str(), ex.what());
          exit(1);
      }

      local_costmap_ptr_->start();

      //ROS_INFO("Local planner loaded");

      initROSComms();      

      ROS_INFO("[%s]: Initialization complete. Entering spin...",ros::this_node::getName().c_str() );
      ros::spin();       
    }

    void local_planner::loadROSParams(){
                      
      // LOAD ROS PARAMETERS ....................................
      std::string temp;
      nodeHandle_.param("robot_id", temp, std::string("1"));
      robot_id_=std::stoi(temp);

      nodeHandle_.param("global_frame", global_frame_, std::string("/world"));    
      nodeHandle_.param("current_task_topic_name", current_task_topic_name_, std::string("/robot"+std::to_string(robot_id_)+"/control/controller/task"));

      nodeHandle_.param("controller_frequency_", temp, std::string("0.5"));
      controller_frequency_=std::stod(temp);
      nodeHandle_.param("local_planner_plugin", local_planner_class_name_, std::string("base_local_planner/TrajectoryPlannerROS"));        
    }
    
    void local_planner::initROSComms(){

      //! ROS subscriber to Tasks..
      current_task_sub_ = nodeHandle_.subscribe(current_task_topic_name_, 1000, &local_planner::taskCallback, this);
      //ROS_INFO("Task subscriber created");
      
      //! ROS subscriber to current plan. This topic is automatically created 
      std::vector<string> strs;
      
      // first is pacakge name and second should be the local planner name: base_local_planner/TrajectoryPlannerROS
      boost::split(strs, local_planner_class_name_, boost::is_any_of("/"));

      // this topic name is hardwired ...
      current_local_plan_topic_name_ = ros::this_node::getName()+strs[1]+"/local_plan";
      current_local_plan_sub_ = nodeHandle_.subscribe(current_local_plan_topic_name_, 1000, &local_planner::localPlanCallback, this);
      //ROS_INFO("Local Path subscriber created");

      ros::Duration durTimer(controller_frequency_);
         
      updateTimer_ = nodeHandle_.createTimer(durTimer,  &local_planner::updateLocalPathCallback,this,false);
    }

    local_planner::~local_planner(){
        if(local_costmap_ptr_ != NULL)
          delete local_costmap_ptr_;

        local_planner_ptr_.reset();    
        ROS_INFO("[%s]: Exiting",ros::this_node::getName().c_str());     
    }

    void local_planner::updateLocalPathCallback(const ros::TimerEvent&t){
      // check if this callback is taking too long
      double expected_dur, real_dur;
      expected_dur = (t.current_expected - t.last_expected).toSec();
      real_dur = (t.current_real - t.last_real).toSec();
      if (real_dur>expected_dur){
        ROS_WARN("[%s]: Programmed update period was (%3.3f) and we got (%3.3f).",ros::this_node::getName().c_str(),expected_dur.toSec(), real_dur.toSec());
      }

      // create local plan
      geometry_msgs::Twist cmd_vel;      
      unsigned int task_len = current_task_.path.path.size();
      
      if (task_len>0){          
          //ROS_WARN("Updating local plan.");
          if(local_planner_ptr_->computeVelocityCommands(cmd_vel)){
                ROS_WARN("[%s]: Got a valid command from the local planner plugin: %.3lf, %.3lf, %.3lf",
                                ros::this_node::getName().c_str(), cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          } else {
                ROS_WARN("[%s]: The local planner plugin could not find a valid plan.",ros::this_node::getName().c_str());
          }
          //ROS_WARN("Updated.");
      }


      /*
      Recreate new Task blending this local path and the global task. This new task will:
        - Start at current robot position (we need subscriber).
        - End at original destination (given by original Task).
        - Follow the local path: 
            * we will find the closest task point to the end of local path.
            * then find out how many task points we have to alter.
            * and resample our local path to replace task points.
      Finally send it to the veh through service ...
      
      */
    }

    void local_planner::localPlanCallback(const nav_msgs::Path::ConstPtr& plan_msg){
      unsigned int plan_len = plan_msg->poses.size();
      ROS_WARN("[%s]: Local plan has: %u points", ros::this_node::getName().c_str(), plan_len);
      ROS_WARN("[%s]: last point is at: %3.3f, %3.3f", ros::this_node::getName().c_str(), plan_msg->poses[plan_len-1].pose.position.x,plan_msg->poses[plan_len-1].pose.position.y);
    }

    void local_planner::taskCallback(const orunav_msgs::Task::ConstPtr& task_msg){
      
      //ROS_WARN("Task received.");
      current_task_ = *task_msg;
      
      unsigned int task_len = current_task_.path.path.size();
      ROS_WARN("[%s]: Task had: %u points", ros::this_node::getName().c_str(), task_len);

      if (task_len>0){
          //ROS_WARN("Casting into path.");
          global_plan_ = this->task2path(current_task_);
          
          ROS_WARN("[%s]: last task point is at: %3.3f, %3.3f", ros::this_node::getName().c_str(), global_plan_[task_len-1].pose.position.x,global_plan_[task_len-1].pose.position.y);              
          //ROS_WARN("Passing to local planner.");
          // pass plan to local planner
          if(!local_planner_ptr_->setPlan(global_plan_)){
            //ABORT and SHUTDOWN COSTMAPS
            ROS_ERROR("[%s]: Failed to pass global plan to the controller, aborting.",ros::this_node::getName().c_str());
          }
      } else {
        ROS_WARN("[%s]: Ignoring len 0 task",ros::this_node::getName().c_str());
      }

      //ROS_WARN("Finished.");
    }

    std::vector<geometry_msgs::PoseStamped> local_planner::task2path(orunav_msgs::Task inTask){
      std::vector<geometry_msgs::PoseStamped> outPath;
      orunav_msgs::PoseSteering psi;
      unsigned int task_len = inTask.path.path.size();

      for (unsigned int i = 0; i < task_len; ++i) {
        psi=inTask.path.path[i];

        geometry_msgs::PoseStamped poseSt;
        poseSt.header.frame_id = global_frame_;
        poseSt.header.stamp = ros::Time::now();
        poseSt.pose = geometry_msgs::Pose(psi.pose);

        outPath.push_back(poseSt);
      }


      return outPath;
    }

    
    
} // end of namespace iliad_local_planner
