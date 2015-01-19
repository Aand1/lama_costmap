/*
 * Node using a Navigating jockey from local map
 *
 * Parameters:
 * navigating_jockey_server_name, String, node_name + "_server"
 * odom_frame, String, "odom", frame of the laser sensor
 * frontier_width, String, no default, how wide must an exit be
 *
*/

#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug

#include <nj_costmap/jockey.h>

// TODO: set these as parameters in the jockey
// const double maxtheta = 1.6;
// const double max_distance = 3.0;
// const double reach_distance = 0.05;

int main(int argc, char **argv)
{
  // Debug log level
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "default_name_nj_costmap");
  ros::NodeHandle nh; // Log in jockey works better with this line.
  ros::NodeHandle private_nh("~");

  /* Minimal frontier width */
  if (!private_nh.hasParam("frontier_width"))
  {
    ROS_ERROR("Parameter %s/frontier_width not set, exiting", private_nh.getNamespace().c_str());
    return 1;
  }
  double frontier_width;
  private_nh.param<double>("frontier_width", frontier_width, 0.0);

  /* Navigating jockey name */
  std::string navigating_jockey_name;
  private_nh.param<std::string>("navigating_jockey_server_name", navigating_jockey_name,
      ros::this_node::getName() + "_server");

  nj_costmap::Jockey jockey(navigating_jockey_name, frontier_width);

  ROS_INFO("%s started (with server %s)", ros::this_node::getName().c_str(), jockey.getName().c_str());
  ros::spin();

  return 0;
}

