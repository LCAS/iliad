#include "tw_controller_node.h"

double v;
double rot;
char message_flags;


void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{    
  v = cmd_vel->linear.x;
  rot = cmd_vel->angular.z;
  ROS_INFO("[zombie@%d] vel command received (%2.2f, %2.2f)", __LINE__,v,rot);
  message_flags = SW_CAN_FLAG_OVERRIDE | SW_CAN_FLAG_DRIVE_ON | SW_CAN_FLAG_STEER_ON;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tw_controller_node");

  ros::NodeHandle n;
    
  double refresh_period_msecs= 20;
  double refresh_rate=1000.0/refresh_period_msecs; //Herzs
  
  ros::Rate r(refresh_rate);

  ros::Subscriber sub = n.subscribe("z_cmd_vel", 1, cmdVelReceived);
  ROS_INFO("[tw_controller_node@%d] ROS subscriber created ", __LINE__);
  
  double _max_car_velocity_change;
  double _max_car_steer_angle_change;
  commandSendero command_sender( _max_car_velocity_change,  _max_car_steer_angle_change);
  ROS_INFO("[tw_controller_node@%d] CAN command sender created ", __LINE__);

  while (n.ok())
  {
      ros::spinOnce();
      command_sender.formWriteMSG(v, rot, message_flags);
	  ROS_INFO(".");
      r.sleep();
  }
  
  ROS_INFO("[tw_controller_node@%d] Program finished. Bye! ", __LINE__);
}

// EOF
