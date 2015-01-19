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

  /* Compulsory parameter: robot radius */
  if (!private_nh.hasParam("robot_radius"))
  {
    ROS_ERROR("Parameter %s/robot_radius not set, exiting", private_nh.getNamespace().c_str());
    return 1;
  }
  double robot_radius;
  private_nh.param<double>("robot_radius", robot_radius, 0.0);

  std::string navigating_jockey_name;
  private_nh.param<std::string>("navigating_jockey_server_name",
      navigating_jockey_name, ros::this_node::getName() + "_server");

  nj_oa_costmap::Jockey jockey(navigating_jockey_name, robot_radius);

  ROS_INFO_STREAM(ros::this_node::getName() << " started (with server " << jockey.getName() << ")");
  ros::spin();
  return 0;
}


