#include "zombienode.h"

double v;
double rot;
char message_flags;


void set20(){
	      v = 0.0;
      rot=0.0;
      message_flags = 0;
}

void timerCallback(const ros::TimerEvent&)
{
	  set20();
      ROS_INFO("[zombie@%d] resetting speeds to (%2.2f, %2.2f)", __LINE__,v,rot);
}


void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
	
  v = cmd_vel->linear.x;
  rot = cmd_vel->angular.z;
  ROS_INFO("[zombie@%d] vel command received (%2.2f, %2.2f)", __LINE__,v,rot);
  //aqui el send y printf
  message_flags = SW_CAN_FLAG_OVERRIDE | SW_CAN_FLAG_DRIVE_ON | SW_CAN_FLAG_STEER_ON;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "zombie_node");

  ros::NodeHandle n;
    
  double refresh_period_msecs= 50;
  double refresh_rate=1000.0/refresh_period_msecs; //Herzs
  
  ros::Rate r(refresh_rate);

  ros::Subscriber sub = n.subscribe("z_cmd_vel", 1, cmdVelReceived);
  ROS_INFO("[zombie@%d] ROS subscriber created ", __LINE__);
  
  double _max_car_velocity_change;
  double _max_car_steer_angle_change;
  commandSendero command_sender( _max_car_velocity_change,  _max_car_steer_angle_change);
  ROS_INFO("[zombie@%d] CAN command sender created ", __LINE__);
  //ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);
  //set20();

  while (n.ok())
  {
      ros::spinOnce();
      command_sender.formWriteMSG(v, rot, message_flags);
      r.sleep();
  }
  
  ROS_INFO("[zombie@%d] Program finished. Bye! ", __LINE__);
}

// EOF
