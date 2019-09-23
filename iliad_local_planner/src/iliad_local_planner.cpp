#include <iliad_local_planner/iliad_local_planner.hpp>


namespace iliad_local_planner {
    
    
    local_planner::local_planner(ros::NodeHandle& n)
    : nodeHandle_(n),
    tf2_listener_(tfBuffer2_),
    listener_(ros::Duration(10)),
    local_planner_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    local_costmap_ptr_(NULL),    
    global_frame_("world"),
    local_planner_class_name_("base_local_planner/TrajectoryPlannerROS")
    {
      // ...................................................................................................
      // LOAD ROS PARAMS
      loadROSParams();

      // See what did we load
      printROSParams();

      // init other stuff
      robot_frame_id_="robot4/base_footprint";
      // ...................................................................................................
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

      // now the local planner is running, we need to keep the costmap updated.
      local_costmap_ptr_->start();

      // ...................................................................................................
      initROSComms();      

      // .....................................................
      // after comm structure is in place, some periodic stuff
      ros::Duration durTimer(1.0/controller_frequency_);         
      
      updateTimer_ = nodeHandle_.createTimer(durTimer,  &local_planner::updateLocalPathCallback,this,false);

      ROS_INFO("[%s]: Initialization complete. Entering spin...",ros::this_node::getName().c_str() );
      ros::spin();       
    }

    void local_planner::loadROSParams(){
                      
      // LOAD ROS PARAMETERS ....................................
      if (!nodeHandle_.getParam("robot_id", robot_id_)){
        robot_id_ = 1;
        ROS_WARN("[%s]: Can't find (%s). Using default",ros::this_node::getName().c_str(),"robot_id");
      }

      if (!nodeHandle_.getParam("global_frame", global_frame_)){
        global_frame_ = "/world";
        ROS_WARN("[%s]: Can't find (%s). Using default",ros::this_node::getName().c_str(),"global_frame");
      }

      if (!nodeHandle_.getParam("current_task_topic_name", current_task_topic_name_)){
        current_task_topic_name_ = "/robot"+std::to_string(robot_id_)+"/control/controller/task";
        ROS_WARN("[%s]: Can't find (%s). Using default",ros::this_node::getName().c_str(),"current_task_topic_name");
      }

      if (!nodeHandle_.getParam("controller_frequency", controller_frequency_)){
        controller_frequency_ = 2.0;
        ROS_WARN("[%s]: Can't find (%s). Using default",ros::this_node::getName().c_str(),"controller_frequency");
      }

      if (!nodeHandle_.getParam("local_planner_plugin", local_planner_class_name_)){
        local_planner_class_name_ ="base_local_planner/TrajectoryPlannerROS";
        ROS_WARN("[%s]: Can't find (%s). Using default",ros::this_node::getName().c_str(),"local_planner_plugin");
      }

      if (!nodeHandle_.getParam("ven_execute_task_srv_client_name", ven_execute_task_srv_client_name_)){
        ven_execute_task_srv_client_name_ ="/robot"+std::to_string(robot_id_)+"/ven_execute_task";
        ROS_WARN("[%s]: Can't find (%s). Using default",ros::this_node::getName().c_str(),"ven_execute_task_srv_client_name");
      }

      if (!nodeHandle_.getParam("coord_execute_task_srv_name", coord_execute_task_srv_name_)){
        coord_execute_task_srv_name_ ="/robot"+std::to_string(robot_id_)+"/execute_task";
        ROS_WARN("[%s]: Can't find (%s). Using default",ros::this_node::getName().c_str(),"coord_execute_task_srv_name");
      }

      if (!nodeHandle_.getParam("robot_odometry_topic_name_", robot_odometry_topic_name_)){        
        robot_odometry_topic_name_ = "/robot"+std::to_string(robot_id_)+"/odom";
        ROS_WARN("[%s]: Can't find (%s). Using default",ros::this_node::getName().c_str(),"robot_odometry_topic_name");
      }

      if (!nodeHandle_.getParam("robot_controller_reports_topic_name", robot_controller_reports_topic_name_)){        
        robot_controller_reports_topic_name_ = "/robot"+std::to_string(robot_id_)+"/control/controller/reports";
        ROS_WARN("[%s]: Can't find (%s). Using default",ros::this_node::getName().c_str(),"robot_controller_reports_topic_name");
      }

      if (!nodeHandle_.getParam("robot_robot_report_topic_name", robot_robot_report_topic_name_)){        
        robot_robot_report_topic_name_ = "/robot"+std::to_string(robot_id_)+"/control/report";
        ROS_WARN("[%s]: Can't find (%s). Using default",ros::this_node::getName().c_str(),"robot_robot_report_topic_name");
      }

    }

    void local_planner::printROSParams(){
      ROS_INFO("[%s]: -----------------------------------------------------.",ros::this_node::getName().c_str());  
      ROS_INFO("[%s]: rosparams.",ros::this_node::getName().c_str());
      ROS_INFO("[%s]: Current nodehandle namespace (%s)",ros::this_node::getName().c_str(), nodeHandle_.getNamespace().c_str());
      ROS_INFO("[%s]: Current node namespace (%s)",ros::this_node::getName().c_str(), ros::this_node::getNamespace().c_str());
      ROS_INFO("[%s]: -----------------------------------------------------.",ros::this_node::getName().c_str());
      ROS_INFO("[%s]: robot_id (%d)",ros::this_node::getName().c_str(),robot_id_);
      ROS_INFO("[%s]: global_frame (%s)",ros::this_node::getName().c_str(),global_frame_.c_str());
      ROS_INFO("[%s]: robot_controller_reports_topic_name (%s)",ros::this_node::getName().c_str(),robot_controller_reports_topic_name_.c_str());
      ROS_INFO("[%s]: robot_robot_report_topic_name (%s)",ros::this_node::getName().c_str(),robot_robot_report_topic_name_.c_str());

      ROS_INFO("[%s]: current_task_topic_name (%s)",ros::this_node::getName().c_str(),current_task_topic_name_.c_str());
      ROS_INFO("[%s]: controller_frequency (%3.3f)",ros::this_node::getName().c_str(),controller_frequency_);
      ROS_INFO("[%s]: local_planner_plugin (%s)",ros::this_node::getName().c_str(),local_planner_class_name_.c_str());
      ROS_INFO("[%s]: ven_execute_task_srv_client_name (%s)",ros::this_node::getName().c_str(),ven_execute_task_srv_client_name_.c_str());
      ROS_INFO("[%s]: coord_execute_task_srv_name (%s)",ros::this_node::getName().c_str(),coord_execute_task_srv_name_.c_str());
      ROS_INFO("[%s]: -----------------------------------------------------.",ros::this_node::getName().c_str());



    }
    
    void local_planner::initROSComms(){

      std::vector<string> planner_info;      
      // first is pacakge name and second should be the local planner name: base_local_planner/TrajectoryPlannerROS
      boost::split(planner_info, local_planner_class_name_, boost::is_any_of("/"));
      

      // TOPIC PUBLISHERS .............................................................................................
      if (local_planner_class_name_ == "teb_local_planner/TebLocalPlannerROS"){
          // this topic name is hardwired ...
          teb_via_points_topic_name_ = ros::this_node::getName()+"/"+planner_info[1]+"/via_points";
          teb_via_points_topic_name_pub_ = nodeHandle_.advertise<nav_msgs::Path>(teb_via_points_topic_name_, 5);          
      }

      //temp_pub_ = nodeHandle_.advertise<nav_msgs::Path>("/debug_local_path", 5);          
                

      // SERVICE CLIENTS ...............................................................................................
      ven_execute_task_srv_client_ = nodeHandle_.serviceClient<orunav_msgs::ExecuteTask>(ven_execute_task_srv_client_name_);

      // TOPIC  SUBSCRIBERS ............................................................................................

      robot_controller_reports_sub_ = nodeHandle_.subscribe(robot_controller_reports_topic_name_, 1000, &local_planner::robot_controller_reports_callback, this);

      robot_robot_report_sub_ = nodeHandle_.subscribe(robot_robot_report_topic_name_, 1000, &local_planner::robot_robot_report_callback, this);      

      robot_odometry_sub_ = nodeHandle_.subscribe(robot_odometry_topic_name_, 1000, &local_planner::robotOdometryCallback, this);
      
      
      //! ROS subscriber to current plan. This topic is automatically created 
      // so, this topic name is hardwired ...
      current_local_plan_topic_name_ = ros::this_node::getName()+"/"+planner_info[1]+"/local_plan";
      current_local_plan_sub_ = nodeHandle_.subscribe(current_local_plan_topic_name_, 1000, &local_planner::localPlanCallback, this);
      //ROS_INFO("Local Path subscriber created");     

     // SERVICES ...................................................................................................
     coord_execute_task_srv_ = nodeHandle_.advertiseService(coord_execute_task_srv_name_, &local_planner::coord_execute_task_callback, this);
      
    }

    local_planner::~local_planner(){
        if(local_costmap_ptr_ != NULL)
          delete local_costmap_ptr_;

        local_planner_ptr_.reset();    
        ROS_INFO("[%s]: Exiting",ros::this_node::getName().c_str());     
    }



    void local_planner::robot_controller_reports_callback(const orunav_msgs::ControllerReport::ConstPtr& report_msg){
        steering_=report_msg->state.steering_angle;
        controller_status_ = report_msg->status;        
    }

    void local_planner::robot_robot_report_callback(const orunav_msgs::RobotReport::ConstPtr& report_msg){        
        robot_status_ = report_msg->status;        
    }

    void local_planner::robotOdometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
      // pose is in the coordinate frame  header.frame_id.
      // twist is in the coordinate frame given by child_frame_id
      // child frame normally is the coordinate frame of the mobile base

        robot_poseSt_.header = odom_msg->header;
        robot_poseSt_.pose = odom_msg->pose.pose;

        robot_speedSt_.header = odom_msg->header;        
        robot_speedSt_.header.frame_id = odom_msg->child_frame_id;        
        robot_speedSt_.twist = odom_msg->twist.twist;

        robot_frame_id_ =  odom_msg->child_frame_id;        

    }

    void local_planner::updateLocalPathCallback(const ros::TimerEvent&t){
      // check if this callback is taking too long
      double expected_dur, real_dur;
      expected_dur = (t.current_expected - t.last_expected).toSec();
      real_dur = (t.current_real - t.last_real).toSec();
      if (real_dur>(1.05*expected_dur)){
        ROS_WARN("[%s]: Programmed update freq was (%3.3f) and we got (%3.3f).",ros::this_node::getName().c_str(),1.0/expected_dur, 1.0/real_dur);
      }

      // create local plan
      geometry_msgs::Twist cmd_vel;      
      unsigned int task_len = current_task_.path.path.size();
      
      if (task_len>0){          
          //ROS_WARN("Updating local plan.");
          if(local_planner_ptr_->computeVelocityCommands(cmd_vel)){
                //ROS_WARN("[%s]: Got a valid command from the local planner plugin: %.3lf, %.3lf, %.3lf",
                //                ros::this_node::getName().c_str(), cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );

          } else {
                ROS_WARN("[%s]: The local planner plugin could not find a valid plan.",ros::this_node::getName().c_str());
          }
          //ROS_WARN("Updated.");
      }


    }

    void local_planner::printSuspicius(std::vector<geometry_msgs::PoseStamped> inV){
      double d;
      for (unsigned int i = 0; i < inV.size(); ++i) {
                d = fmax(inV[i].pose.position.x, fmax(inV[i].pose.position.y, inV[i].pose.position.z) );
                if ( d >200.0){                  
                  ROS_ERROR("[%s]: //////////////////////////////////////////////////",ros::this_node::getName().c_str());
                  ROS_INFO("[%s]: Weird pose magnitude!",ros::this_node::getName().c_str());
                  ROS_INFO("[%s]: pose [%u] (%3.1f, %3.1f, %3.1f).",ros::this_node::getName().c_str(),i, inV[i].pose.position.x, inV[i].pose.position.y, inV[i].pose.position.z);                  
                  ROS_ERROR("[%s]: //////////////////////////////////////////////////",ros::this_node::getName().c_str());
                }
      }
    }

    
    void local_planner::printPoseStamped(std::vector<geometry_msgs::PoseStamped> inV){      
      ROS_INFO("[%s]: Lenght (%lu)",ros::this_node::getName().c_str(),inV.size());
      for (unsigned int i = 0; i < inV.size(); ++i) {

          ROS_INFO("[%s]: pose [%u] (%3.1f, %3.1f, %3.1f deg.).",ros::this_node::getName().c_str(),
                                                              i, inV[i].pose.position.x, 
                                                                 inV[i].pose.position.y, 
                                                                 180.0*getYaw(inV[i].pose.orientation)/3.141592);                  
      }
    }

    void local_planner::printPoseSteering(std::vector<orunav_msgs::PoseSteering> inV){      
      ROS_INFO("[%s]: Lenght (%lu)",ros::this_node::getName().c_str(),inV.size());
      for (unsigned int i = 0; i < inV.size(); ++i) {
          ROS_INFO("[%s]: pose [%u] (%3.1f, %3.1f, %3.1f deg.).(%3.3f deg.)",ros::this_node::getName().c_str(),
                                                                    i, inV[i].pose.position.x, 
                                                                       inV[i].pose.position.y, 
                                                                       180.0*getYaw(inV[i].pose.orientation)/3.141592,  
                                                                       180.0*(inV[i].steering)/3.141592);                  
      }
    }


    double local_planner::getYaw(geometry_msgs::Quaternion q0){
       tf::Quaternion q(q0.x, q0.y, q0.z, q0.w);
       tf::Matrix3x3 m(q);
       double roll, pitch, yaw;
       m.getRPY(roll, pitch, yaw);
       return yaw;
    }


    bool local_planner::isNewLocalPlan(const nav_msgs::Path::ConstPtr& plan_msg){
        bool ans = true;

        nav_msgs::Path old_plan; new_plan;
        if (last_local_plan_!= NULL){

          // find starting point in path (both)

          // crop path from starting point to end (both)

          // resample to have same number of points


        }

        last_local_plan_ = plan_msg;
        return ans;
    }

    void local_planner::localPlanCallback(const nav_msgs::Path::ConstPtr& plan_msg){

      unsigned int local_plan_len = plan_msg->poses.size();
      ROS_WARN("[%s]: Received local plan:", ros::this_node::getName().c_str());      
      printPoseStamped(plan_msg->poses);
      ROS_WARN("[%s]: ................................", ros::this_node::getName().c_str());      

      if (isNewLocalPlan(plan_msg)){

      //ROS_WARN("[%s]: last point is at: %3.3f, %3.3f", ros::this_node::getName().c_str(), plan_msg->poses[local_plan_len-1].pose.position.x,plan_msg->poses[local_plan_len-1].pose.position.y);
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

      // Find closest point to robot pose in local path: start_local_path_i
      double min_dist = INFINITY;
      double dist ;
      unsigned int  start_local_path_i = local_plan_len;

      for (unsigned int i = 0; i < local_plan_len; ++i) {
        dist = getDist(plan_msg->poses[i], robot_poseSt_ );
        if (dist< min_dist){
          min_dist = dist;
          start_local_path_i = i;
        }

      }
      
      ROS_INFO("[%s]: Robot at [%u] of local plan (%3.1f, %3.1f, %3.1f deg.).",ros::this_node::getName().c_str(),start_local_path_i,
                                                                 robot_poseSt_.pose.position.x, 
                                                                 robot_poseSt_.pose.position.y, 
                                                                 180.0*getYaw(robot_poseSt_.pose.orientation)/3.141592);        

      // Find closest point to last local path pose in global task: end_insert_task_i
      // Find closest point to robot pose in global task: start_insert_task_i

      unsigned int task_len = global_path_.poses.size();
      unsigned int end_insert_task_i = task_len;
      unsigned int start_insert_task_i = task_len;
      double min_dist_end_insert = INFINITY;
      double dist_end_insert ;
      double min_dist_start_insert = INFINITY;
      double dist_start_insert ;
      for (unsigned int i = 0; i < task_len; ++i) {
        dist_end_insert = getDist(global_path_.poses[i], plan_msg->poses[local_plan_len-1] );
        if (dist_end_insert< min_dist_end_insert){
          min_dist_end_insert = dist_end_insert;
          end_insert_task_i = i;
        }

        dist_start_insert = getDist(global_path_.poses[i], robot_poseSt_ );
        if (dist_start_insert< min_dist_start_insert){
          min_dist_start_insert = dist_start_insert;
          start_insert_task_i = i;
        }
      }
      
      ROS_WARN("[%s]: We're inserting local plan between Task poses:", ros::this_node::getName().c_str());
      ROS_INFO("[%s]: \t Start pose [%u] (%3.1f, %3.1f, %3.1f deg.).",ros::this_node::getName().c_str(),
                                                              start_insert_task_i, global_path_.poses[start_insert_task_i].pose.position.x, 
                                                                 global_path_.poses[start_insert_task_i].pose.position.y, 
                                                                 180.0*getYaw(global_path_.poses[start_insert_task_i].pose.orientation)/3.141592);     
      ROS_INFO("[%s]: \t End pose [%u] (%3.1f, %3.1f, %3.1f deg.).",ros::this_node::getName().c_str(),
                                                              end_insert_task_i, global_path_.poses[end_insert_task_i].pose.position.x, 
                                                                 global_path_.poses[end_insert_task_i].pose.position.y, 
                                                                 180.0*getYaw(global_path_.poses[end_insert_task_i].pose.orientation)/3.141592);     


      ROS_WARN("[%s]: ................................", ros::this_node::getName().c_str());      
      ROS_WARN("[%s]: Task", ros::this_node::getName().c_str());      
      printPoseStamped(global_path_.poses);
      ROS_WARN("[%s]: ................................", ros::this_node::getName().c_str());      

      // Find how many task points we have to replace:  end_insert_task_i - start_insert_task_i = n_points
      unsigned int n_points = end_insert_task_i - start_insert_task_i + 1 ;


      orunav_msgs::ExecuteTask::Request new_req;
      // hope this makes a copy... of the current task...
      new_req.task = current_task_;

      if (false){
      if ((end_insert_task_i>(start_insert_task_i)) || (!current_task_.update)){
          ROS_WARN("[%s]: Our local plan has to be subsampled to [%u] points", ros::this_node::getName().c_str(), n_points);

          // Trim local path to start at start_local_path_i
          nav_msgs::Path new_local_path;
          new_local_path.header =plan_msg->header;
          for (unsigned int i = start_local_path_i; i < local_plan_len; ++i) {
            new_local_path.poses.push_back(plan_msg->poses[i]);
          }

          printSuspicius(new_local_path.poses);

          // subsample new local path to have n_points
          nav_msgs::Path new_new_local_path = linear_subsample(new_local_path, n_points);
          ROS_WARN("[%s]: Final Local plan", ros::this_node::getName().c_str());
          printPoseStamped(new_new_local_path.poses);
          


          // cast path poses into oru style
          std::vector<orunav_msgs::PoseSteering> local_path_oru = cast2PoseSteering(new_new_local_path.poses,1.2);


          
          // TODO create new task: using first n points in path
          

          // second change, we start here
          new_req.task.target.start.pose = robot_poseSt_.pose;
          new_req.task.target.start.steering = steering_;

          //ROS_WARN("[%s]: Task copy has [%lu] points", ros::this_node::getName().c_str(), new_req.task.path.path.size());
          //ROS_WARN("[%s]: Posesteering list has [%lu] points", ros::this_node::getName().c_str(), local_path_oru.size());
          // third change: first few points         
          for (unsigned int i = start_insert_task_i; i<=end_insert_task_i ; i++) {
            //ROS_WARN("[%s]: i [%u]", ros::this_node::getName().c_str(), i);
            new_req.task.path.path[i] = local_path_oru[i-start_insert_task_i];
          }
          
          ROS_WARN("[%s]: We are sending the robot THIS trajectory", ros::this_node::getName().c_str());
          printPoseSteering(new_req.task.path.path);
          ROS_WARN("[%s]: ................................", ros::this_node::getName().c_str());      

          //TODO  new_req.task.criticalPoint needs to be changed? 

          // future calls to this method will perform updates on the task.
          
          
          current_task_ = new_req.task;
          current_task_.update = true;

      } else {
          ROS_WARN("[%s]: Robot is already at point [%u] of task and we should be inserting local at  point [%u]. Skipping ", ros::this_node::getName().c_str(), start_insert_task_i, end_insert_task_i);
      }
      }

      // TODO Call Service ...
      orunav_msgs::ExecuteTask msg;
      msg.request = new_req;
      if (!current_task_.update){
        current_task_.update = true;
      }

          if (ven_execute_task_srv_client_.call(msg)) {
              ROS_INFO("[%s] - execute task successful", ros::this_node::getName().c_str());
          }else{
              ROS_ERROR("[%s] - Call to service execute_task returns ERROR", ros::this_node::getName().c_str());              
          }

      }
    }

    bool local_planner::coord_execute_task_callback(orunav_msgs::ExecuteTask::Request &req, orunav_msgs::ExecuteTask::Response &res){
      //ROS_WARN("Task received.");
      current_task_ = req.task;
      task_timestamp_ = ros::Time::now();

      unsigned int task_len = current_task_.path.path.size();
      //ROS_WARN("[%s]: Task ", ros::this_node::getName().c_str());
      //printPoseSteering(current_task_.path.path);
      

      if (task_len>0){
          //ROS_WARN("Casting into path.");
          global_plan_ = this->task2poseStVect(current_task_);
          //ROS_WARN("[%s]: Task (again)", ros::this_node::getName().c_str());
          //printPoseStamped(global_plan_);
          global_path_ = this->poseStVect2Path(global_plan_);
          //ROS_WARN("[%s]: last task point is at: %3.3f, %3.3f", ros::this_node::getName().c_str(), global_plan_[task_len-1].pose.position.x,global_plan_[task_len-1].pose.position.y);              
          //ROS_WARN("Passing to local planner.");

          // if planner is teb... enforce via paths
          if (local_planner_class_name_ == "teb_local_planner/TebLocalPlannerROS"){   
              // teb will try to stick to these via points as much as config requires               
              teb_via_points_topic_name_pub_.publish(global_path_);  //viaPoints);
          }          

          // pass plan to local planner
          if(!local_planner_ptr_->setPlan(global_plan_)){
            //ABORT and SHUTDOWN COSTMAPS
            ROS_ERROR("[%s]: Failed to pass global plan to the controller, aborting.",ros::this_node::getName().c_str());
            res.result = 0;             
          }

      } else {
        ROS_WARN("[%s]: Ignoring len 0 task",ros::this_node::getName().c_str());
        res.result = 0; 
      }

      res.result = 1; 
      return (res.result==1);

      //ROS_WARN("Finished.");
    }

    std::vector<geometry_msgs::PoseStamped> local_planner::task2poseStVect(orunav_msgs::Task inTask){
      std::vector<geometry_msgs::PoseStamped> outPoseVect;
      orunav_msgs::PoseSteering psi;
      unsigned int task_len = inTask.path.path.size();

      for (unsigned int i = 0; i < task_len; ++i) {
        psi=inTask.path.path[i];

        geometry_msgs::PoseStamped poseSt;
        poseSt.header.frame_id = global_frame_;
        poseSt.header.stamp = ros::Time::now();
        poseSt.pose = geometry_msgs::Pose(psi.pose);

        outPoseVect.push_back(poseSt);
      }
      return outPoseVect;
    }

    nav_msgs::Path local_planner::poseStVect2Path(std::vector<geometry_msgs::PoseStamped> inPoseStVect){
      nav_msgs::Path outPath;
      
      outPath.header.frame_id = inPoseStVect[0].header.frame_id;
      outPath.poses = inPoseStVect;

      return outPath;

    }

    double local_planner::getDist(geometry_msgs::PoseStamped poseStA, geometry_msgs::PoseStamped poseStB){

        double dist = HUGE_VAL;
    
        if (poseStA.header.frame_id!=poseStB.header.frame_id){
            poseStA=transformPose(poseStB.header.frame_id, poseStA);
        }

        dist = sqrt( pow(poseStA.pose.position.x-poseStB.pose.position.x, 2) +
                     pow(poseStA.pose.position.y-poseStB.pose.position.y, 2) +
                     pow(poseStA.pose.position.z-poseStB.pose.position.z, 2) );
        
        return dist;
    }

    std::string purgueSlash(std::string inStr){
          std::string ans;              
          ans =  std::string(inStr);                
          if (ans.front() == '/'){            
            ans =  ans.erase(0,1);
          } 

          // if (ans == ""){
          //   ROS_FATAL("[%s]: INVALID frame_id (%s)",ros::this_node::getName().c_str(),ans.c_str());
          //   exit(1);
          // }
          
          return ans;              
    }

    geometry_msgs::PoseStamped local_planner::transformPose(std::string frame_id, geometry_msgs::PoseStamped poseIn){

          // stupid tf1 version ...
          // try{

          //   listener_.waitForTransform(poseStB.header.frame_id,poseStA.header.frame_id,ros::Time(0), ros::Duration(5));
          //   listener_.lookupTransform(poseStB.header.frame_id, poseStA.header.frame_id, ros::Time(0), transform);
            
          //   //listener_.transformPose(	poseStB.header.frame_id,poseStA,poseStA);	       
          // }catch(tf::TransformException e){
          //   ROS_ERROR("Failed to cast pose from (%s) to (%s), skipping.\nReason: (%s)", poseStA.header.frame_id.c_str(), poseStB.header.frame_id.c_str() ,e.what());
          //   return dist;
          // }

          geometry_msgs::PoseStamped poseOut;

          frame_id = purgueSlash(frame_id);
          poseIn.header.frame_id = purgueSlash(poseIn.header.frame_id);
          
          try{
              pose_to_frame_tf_ = tfBuffer2_.lookupTransform(frame_id,  poseIn.header.frame_id, ros::Time(0), ros::Duration(1.0) );
              tf2::doTransform(poseIn, poseOut, pose_to_frame_tf_); 
          }catch(tf::TransformException e){
            ROS_ERROR("[%s]: Failed to cast pose from (%s) to (%s), skipping.\nReason: (%s)",ros::this_node::getName().c_str(), poseIn.header.frame_id.c_str(), frame_id.c_str() ,e.what());
          }
          return poseOut;

    }

    nav_msgs::Path local_planner::linear_subsample(nav_msgs::Path inputPath, unsigned int outLen){
        nav_msgs::Path outPath;
        outPath.header = inputPath.header;        

        unsigned int lastInIndex, lastOutIndex, indexFactor, index;
        // constant values to speed up process...
        lastInIndex = inputPath.poses.size()-1;
        lastOutIndex = (outLen-1);        

        if (outLen>1){
            indexFactor = lastInIndex  / lastOutIndex;
            
            outPath.poses.push_back(inputPath.poses[0]); 

            for (unsigned int i = 1; i < outLen; ++i) {
              geometry_msgs::PoseStamped res;
              res.header.frame_id = inputPath.poses[0].header.frame_id;

              index = min(lastInIndex-1,i * indexFactor);
                           
              res.pose.position.x = (inputPath.poses[index + 1].pose.position.x + inputPath.poses[index].pose.position.x) / 2.0;
              res.pose.position.y = (inputPath.poses[index + 1].pose.position.y + inputPath.poses[index].pose.position.y) / 2.0;
              res.pose.position.z = (inputPath.poses[index + 1].pose.position.z + inputPath.poses[index].pose.position.z) / 2.0;

              if (i>0){
                if ( getDist(res,outPath.poses[outPath.poses.size()-1]) >2.0){                  
                  ROS_ERROR("[%s]: //////////////////////////////////////////////////",ros::this_node::getName().c_str());
                  ROS_INFO("[%s]: Weird cast!",ros::this_node::getName().c_str());
                  ROS_INFO("[%s]: inputPath [%u] (%3.1f, %3.1f, %3.1f).",ros::this_node::getName().c_str(),index, inputPath.poses[index].pose.position.x, inputPath.poses[index].pose.position.y, inputPath.poses[index].pose.position.z);
                  ROS_INFO("[%s]: inputPath [%u] (%3.1f, %3.1f, %3.1f).",ros::this_node::getName().c_str(),index+ 1, inputPath.poses[index+ 1].pose.position.x, inputPath.poses[index+ 1].pose.position.y, inputPath.poses[index+ 1].pose.position.z);              
                  ROS_INFO("[%s]: Resulting into (%3.1f, %3.1f, %3.1f).",ros::this_node::getName().c_str(),res.pose.position.x,res.pose.position.y,res.pose.position.z);
                  ROS_INFO("[%s]: Which goes after (%3.1f, %3.1f, %3.1f).",ros::this_node::getName().c_str(),outPath.poses[outPath.poses.size()-1].pose.position.x,outPath.poses[outPath.poses.size()-1].pose.position.y,outPath.poses[outPath.poses.size()-1].pose.position.z);
                  ROS_ERROR("[%s]: //////////////////////////////////////////////////",ros::this_node::getName().c_str());
                }
              }

              res.pose.orientation.x = (inputPath.poses[index + 1].pose.orientation.x + inputPath.poses[index].pose.orientation.x) / 2.0;
              res.pose.orientation.y = (inputPath.poses[index + 1].pose.orientation.y + inputPath.poses[index].pose.orientation.y) / 2.0;
              res.pose.orientation.z = (inputPath.poses[index + 1].pose.orientation.z + inputPath.poses[index].pose.orientation.z) / 2.0;
              res.pose.orientation.w = (inputPath.poses[index + 1].pose.orientation.w + inputPath.poses[index].pose.orientation.w) / 2.0;

              outPath.poses.push_back(res);
            }
        }
        if (outPath.poses.size()<outLen){
          outPath.poses.push_back(inputPath.poses[lastInIndex]);  // done outside of loop to avoid out of bound access 
        }
        return outPath;
    }

    void local_planner::printAllROSparams(){
      std::vector< std::string > keys;
      nodeHandle_.getParamNames(keys);
      unsigned int keys_len = keys.size();
      ROS_INFO("[%s]: Available rosparams.",ros::this_node::getName().c_str());
      for (unsigned int i = 0; i < keys_len; ++i) {
                ROS_INFO("[%s]:\t %s",ros::this_node::getName().c_str(), keys[i].c_str());
      }

    }
    
    std::vector<double> local_planner::curvature(std::vector<geometry_msgs::PoseStamped> path) {
          // https://stackoverflow.com/questions/32629806/how-can-i-calculate-the-curvature-of-an-extracted-contour-by-opencv

          std::vector<double> curvature( path.size() );

          double curvature2D, divisor;

          geometry_msgs::Point pos, posOld, posOlder;
          geometry_msgs::Point f1stD, f2ndD;   

          posOld = posOlder = path[0].pose.position; 
          for (unsigned int i = 0; i < path.size(); i++ ){
            pos = path[i].pose.position;

            f1stD.x =   pos.x -       posOld.x;
            f1stD.y =   pos.y -       posOld.y;
            f2ndD.x = - pos.x + 2.0 * posOld.x - posOlder.x;
            f2ndD.y = - pos.y + 2.0 * posOld.y - posOlder.y;

            curvature2D =  std::numeric_limits<double>::infinity();
            divisor = f2ndD.x + f2ndD.y;

            if ( std::abs(divisor) > 10e-4 ){
                curvature2D = std::abs( f2ndD.y*f1stD.x - f2ndD.x*f1stD.y ) / 
                                   pow( divisor, 1.5 )  ;
            }

            curvature[i] = curvature2D;

            posOlder = posOld;
            posOld = pos;
          }

          return curvature;
    } 

    std::vector<double> local_planner::steeringAngles(std::vector<geometry_msgs::PoseStamped> path, double wheelbase){

      std::vector<double> steerings = curvature(path);
      double finite_curv = 0.0;

      // get first finite one
      for(int i = 0; i<steerings.size(); i++){        
        if (! isinf(steerings[i]) ) {
          finite_curv = steerings[i];
          break;
        }
      }

      for(int i = 0; i<steerings.size(); i++){        
        if (! isinf(steerings[i]) ) {
          finite_curv = steerings[i];
        }
        steerings[i] = atan(finite_curv * wheelbase);        
      }
      return steerings;
    }

    std::vector<orunav_msgs::PoseSteering> local_planner::cast2PoseSteering(std::vector<geometry_msgs::PoseStamped> path_ros, double wheelbase){
      std::vector<orunav_msgs::PoseSteering> path_oru;
      std::vector<double> steering_angles;
      
      steering_angles = steeringAngles(path_ros, wheelbase);
      
      // if (robot_frame_id_==""){
      //     ROS_ERROR("[%s]: Empty robot frame",ros::this_node::getName().c_str());
      // }

      for(int i = 0; i<path_ros.size(); i++){        
          orunav_msgs::PoseSteering p_oru;

          // if (path_ros[i].header.frame_id==""){
          //     ROS_ERROR("[%s]: Empty pose frame",ros::this_node::getName().c_str());
          // }

          geometry_msgs::PoseStamped global_pose=transformPose(global_frame_, path_ros[i]);

          p_oru.pose = global_pose.pose;
          p_oru.steering = steering_angles[i];
          path_oru.push_back(p_oru);      
      }
      return path_oru;
    }

} // end of namespace iliad_local_planner
