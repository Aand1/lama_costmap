#include <nj_oa_costmap/jockey.h>

namespace nj_oa_costmap {

const double Jockey::fake_laser_beam_count_ = 20; // Must be at least 2.
const double Jockey::range_max_ = 5;

Jockey::Jockey(const std::string& name, const double robot_radius) :
  nj_oa_laser::Jockey(name, robot_radius),
  base_laser_frame_("base_laser_link"),
  twist_handler_(robot_radius, "base_laser_link")
{
  nj_oa_laser::Jockey::initTwistHandlerParam(twist_handler_);
}

void Jockey::onTraverse()
{
  ROS_DEBUG("%s: Received action TRAVERSE or CONTINUE", ros::this_node::getName().c_str());

  ros::Subscriber map_handler = private_nh_.subscribe<nav_msgs::OccupancyGrid>("local_map", 1, &Jockey::handleMap, this);
  pub_twist_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  ros::Rate r(100);
  while (ros::ok())
  {
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      server_.setPreempted();
      break;
    }

    ros::spinOnce();
    r.sleep();
  }
}

void Jockey::handleMap(const nav_msgs::OccupancyGridConstPtr& msg)
{
  geometry_msgs::Twist twist = twist_handler_.getTwist(*msg);

  pub_twist_.publish(twist);
}

} // namespace nj_oa_costmap

