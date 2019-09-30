#include <iliad_oru_local_planner/iliad_oru_local_planner.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "iliad_local_planner_node");
  /*
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
  {
   ros::console::notifyLoggerLevelsChanged();
  }
  */
  ros::NodeHandle nd("~");
  iliad_oru_local_planner::local_planner rg(nd);
  
  
  ros::requestShutdown();
  return 0;
}
