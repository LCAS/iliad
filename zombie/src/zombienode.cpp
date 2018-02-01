#include "zombienode.h"

commandSender *command_sender;

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
  ROS_INFO("[zombie@%d] vel command received (%2.2f, %2.2f)", __LINE__,cmd_vel->linear.x, cmd_vel->angular.z);
  //aqui el send y printf
  command_sender->formWriteMSG(cmd_vel->linear.x, cmd_vel->angular.z);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "zombie_node");

  ros::NodeHandle n;
    
  double refresh_period_msecs= 1000;
  double refresh_rate=1000.0/refresh_period_msecs; //Herzs
  
  ros::Rate r(refresh_rate);

  ros::Subscriber sub = n.subscribe("z_cmd_vel", 1, cmdVelReceived);
  ROS_INFO("[zombie@%d] ROS subscriber created ", __LINE__);
  
  double _max_car_velocity_change;
  double _max_car_steer_angle_change;
  command_sender = new commandSender( _max_car_velocity_change,  _max_car_steer_angle_change);
  ROS_INFO("[zombie@%d] CAN command sender created ", __LINE__);

  while (n.ok())
  {
      r.sleep();
	  //aqui algo para ver que vive
      ROS_INFO("[zombie@%d] ping! ", __LINE__);
  }
  
  ROS_INFO("[zombie@%d] Program finished. Bye! ", __LINE__);
}

// EOF
