/**
 * Localizing jockey based on a local costmap (costmap position is relative to
 * the sensor but orientation is absolute).
 */


#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

#include <lj_costmap/jockey.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localizing_jockey");
  ros::NodeHandle nh("~");
  
  // Change log level.
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  std::string localizing_jockey_server;
  std::string default_server_name = ros::this_node::getName();
  default_server_name += "_server";
	nh.param<std::string>("localizing_jockey_server_name", localizing_jockey_server, default_server_name);

  /* Minimal frontier width */
  if (!nh.hasParam("frontier_width"))
  {
    ROS_ERROR("Parameter %s/frontier_width not set, exiting", nh.getNamespace().c_str());
    return 1;
  }
  double frontier_width;
  nh.param<double>("frontier_width", frontier_width, 0.0);

  lj_costmap::Jockey jockey(localizing_jockey_server, frontier_width);

  ROS_INFO_STREAM(ros::this_node::getName() << " started (with action server " << jockey.getName() << ")");
  ros::spin();
  return 0;
}

