#include <string>

#include <ros/console.h>

#include <nj_oa_costmap/jockey.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nj_oa_costmap");
  ros::NodeHandle private_nh("~");
  
  // Debug log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  /* Compulsory parameter: robot width */
  if (!private_nh.hasParam("robot_width"))
  {
    ROS_ERROR("Parameter %s/robot_width not set, exiting", private_nh.getNamespace().c_str());
    return 1;
  }
  double robot_width;
  private_nh.param<double>("robot_width", robot_width, 0.0);

  std::string navigating_jockey_name;
  std::string default_jockey_name = ros::this_node::getName();
  default_jockey_name += "_server";
	private_nh.param<std::string>("navigating_jockey_server_name", navigating_jockey_name, default_jockey_name);

  lama::nj_oa_costmap::Jockey jockey(navigating_jockey_name, robot_width);

  ROS_INFO("%s started (with server %s)", ros::this_node::getName().c_str(), jockey.getName().c_str());
  ros::spin();
  return 0;
}


