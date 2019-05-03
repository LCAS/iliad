
/*
 */

#include "constrain_maps/constrain_plotter.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "constrain_maps_node");
/*
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
  {
   ros::console::notifyLoggerLevelsChanged();
 }*/

  ros::NodeHandle nd("~");
  constrain_maps::constrain_plotter rg(nd);


  ros::requestShutdown();
  return 0;
}
