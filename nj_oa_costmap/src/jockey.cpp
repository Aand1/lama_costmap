#include <nj_oa_costmap/jockey.h>

namespace nj_oa_costmap
{

Jockey::Jockey(const std::string& name, const double robot_radius) :
  nj_oa_laser::Jockey(name, robot_radius),
  twist_handler_(robot_radius, "base_laser_link")
{
  initTwistHandlerParam(twist_handler_);
}

void Jockey::initTwistHandlerParam(TwistHandler& twist_handler)
{
  nj_oa_laser::Jockey::initTwistHandlerParam(twist_handler_);

  std::string laser_frame;
  if (private_nh_.getParam("laser_frame", laser_frame))
  {
    twist_handler.laser_frame = laser_frame;
  }

  int fake_laser_beam_count;
  if (private_nh_.getParam("fake_laser_beam_count", fake_laser_beam_count))
  {
    if (fake_laser_beam_count > 1)
    {
      twist_handler.fake_laser_beam_count = fake_laser_beam_count;
    }
    else
    {
      ROS_ERROR_STREAM("Parameter " << private_nh_.getNamespace() << "/fake_laser_beam_count must be at least 2, setting to default");
    }
  }

  double range_max;
  if (private_nh_.getParam("range_max", range_max))
  {
    twist_handler.range_max = range_max;
  }
}

void Jockey::onTraverse()
{
  ROS_DEBUG("Received action TRAVERSE or CONTINUE");

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

