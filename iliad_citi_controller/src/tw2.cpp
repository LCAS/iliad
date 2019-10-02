

#include "tw2.h"
#include <ros/console.h>
#include <orunav_msgs/ControllerReport.h>
#include <orunav_conversions/conversions.h>

double x_inc = 0.0;
double y_inc = 0.0;
orunav_generic::State2d currentState2d_ ;
bool validState2d = false ;
boost::mutex state_mutex_;
boost::mutex cmd_mutex_;
double steps_in_control_loop;
ros::Publisher commands_pub ;
ros::Publisher trajectories_pub;

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{    
  cmd_mutex_.lock();    
  x_inc = cmd_vel->linear.x;
  y_inc = cmd_vel->angular.z;
  cmd_mutex_.unlock();    
  ROS_DEBUG("[tw_controller_node@%d] joy command received (%2.2f, %2.2f)", __LINE__,x_inc,y_inc);  
}


void report_cb(const orunav_msgs::ControllerReportConstPtr &msg) 
{    
    orunav_generic::State2d state = orunav_conversions::createState2dFromControllerStateMsg(msg->state);
    state_mutex_.lock();    
    currentState2d_ = state;
    validState2d_ = true;
    state_mutex_.unlock();
    ROS_DEBUG("[tw_controller_node@%d] report received ", __LINE__);  
}     

void  control_loop()
{

// get x,y,state ..........................
  cmd_mutex_.lock();    
  double xi = x_inc;    
  double yi = y_inc;
  x_inc=0.0;
  y_inc=0.0;
  cmd_mutex_.unlock();  

  state_mutex_.lock();    
  orunav_generic::State2d state = currentState2d_;
  state_mutex_.unlock();


  // create a path
  orunav_generic::PathInterface path = createPath(xi,yi,state, steps_in_control_loop);

  // create a trajectory from the path
  orunav_msgs::ControllerTrajectoryChunkVec  traj = createTraj( path);  
  
  // send it
  trajectories_pub.publish(traj);

}

orunav_generic::PathInterface createPath(double xi, double yi, orunav_generic::State2d curr_state)
{
  orunav_generic::Path path;
  path.addState2dInterface(state);
  double xe = state.getPose2d()[0] + xi;
  double ye = state.getPose2d()[1]+ yi;
  double the = std::atan2(yi,xi);
  double steering = 0.0;
  
  orunav_generic::State2d stateEnd(orunav_generic::Pose2d(xe, ye, the), steering);
  path.addState2dInterface(stateEnd);

  return path;
}


orunav_msgs::ControllerTrajectoryChunkVec  createTraj(const orunav_generic::PathInterface & path) 
{
//orunav_conversions/include/orunav_conversions/conversions.h:160:
//  orunav_msgs::Trajectory createTrajectoryMsgFromTrajectoryInterface(const orunav_generic::TrajectoryInterface &traj)

// orunav_generic/include/orunav_generic/path_utils.h:368:
// Trajectory convertPathToTrajectoryWithoutModel(const PathInterface &path, double dt) 

  orunav_generic::TrajectoryInterface traj =  orunav_generic::convertPathToTrajectoryWithoutModel(path, 0.06); 

  orunav_msgs::ControllerTrajectoryChunkVec c_vec;

  orunav_msgs::ControllerTrajectoryChunk c = orunav_conversions::createControllerTrajectoryChunkFromTrajectoryInterface(traj);

  c.robot_id = robot_id_;
  c.traj_id = 0;
  c.sequence_num = 0;
  c.final = true;
  c_vec.chunks.push_back(c);

  return c_vec;

}

void debugMode()
{    
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tw2");

  ros::NodeHandle n;
  bool isDebugMode;
  int robot_id;
  std::string reports_topic;
  std::string commands_topic;
  std::string twist_topic;
  std::string trajectories_topic;


  // load ROS params ....................................
  n.param<bool>("showDebug", isDebugMode, false);
  if (debugMode){
	  debugMode();
  }
	
  n.param<int>("robot_id", robot_id, 4);
  n.param<std:string>("reports_topic", reports_topic, "/robot4/control/controller/reports"); 
  n.param<std:string>("commands_topic", commands_topic, "/robot4/control/controller/commands"); 
  n.param<std:string>("twist_topic", twist_topic, "/cmd_vel") ;
  n.param<std:string>("trajectories_topic", trajectories_topic, "/robot4/control/controller/trajectories");
  n.param<double>("steps_in_control_loop", steps_in_control_loop, 5.0);




  // ROS subscribers ....................................
  
  // get speed from joystick
  ros::Subscriber twist_sub = n.subscribe(twist_topic, 1, twist_cb);
  ROS_DEBUG("[tw_controller_node@%d] ROS twist subscriber created ", __LINE__);

  // get control msgs
  ros::Subscriber  reports_sub = n.subscribe<orunav_msgs::ControllerReport>(reports_topic, 1,reports_cb);
  ROS_DEBUG("[tw_controller_node@%d] ROS report subscriber created ", __LINE__);



  // ROS publishers  ....................................

  // command 
  commands_pub = n.advertise<orunav_msgs::ControllerTrajectoryChunkVec>(commands_topic,1000);
  ROS_DEBUG("[tw_controller_node@%d] ROS trajectory publisher created ", __LINE__);

  // trajectory
  trajectories_pub = n.advertise<orunav_msgs::ControllerTrajectoryChunkVec>(trajectories_topic,1000);
  ROS_DEBUG("[tw_controller_node@%d] ROS trajectory publisher created ", __LINE__);

  // according to oru each step is 60 ms
  double refresh_rate=1000.0/(steps_in_control_loop * 60.0);   
  ros::Rate r(refresh_rate);

  // Execute control loop
  while (n.ok())
  {
      ros::spinOnce();
      control_loop();
      ROS_INFO(".");
      r.sleep();
  }
  
  ROS_INFO("[tw_controller_node@%d] Program finished. Bye! ", __LINE__);
}

// EOF
