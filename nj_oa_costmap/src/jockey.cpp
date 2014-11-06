#include <nj_oa_costmap/jockey.h>

namespace lama {
namespace nj_oa_costmap {

const double Jockey::fake_laser_beam_count_ = 20; // Must be at least 2.
const double Jockey::range_max_ = 5;

Jockey::Jockey(const std::string& name, const double robot_width) :
  nj_oa_laser::Jockey(name, robot_width),
  base_laser_frame_("base_laser_link")
{
  private_nh_.getParam("laser_frame", base_laser_frame_);
}

void Jockey::onTraverse()
{
  ROS_DEBUG("%s: Received action TRAVERSE or CONTINUE", ros::this_node::getName().c_str());

  ros::Subscriber map_handler = private_nh_.subscribe<nav_msgs::OccupancyGrid>("local_map", 1, &Jockey::handleMap, this);
  pub_twist_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  
  ros::Rate r(100);
  while (true)
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
  // Get the rotation between map and the LaserScan messages that were
  // used to build the map.
  tf::TransformListener tf_listerner;
  tf::StampedTransform tr;
  try
  {
    tf_listerner.waitForTransform(base_laser_frame_, msg->header.frame_id,
        msg->header.stamp, ros::Duration(0.5));
    tf_listerner.lookupTransform(base_laser_frame_, msg->header.frame_id,
        msg->header.stamp, tr);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  const double theta = tf::getYaw(tr.getRotation());

  sensor_msgs::LaserScan scan;
  scan.angle_min = -M_PI_2 - theta;
  scan.angle_max = M_PI_2 - theta;
  scan.angle_increment = (scan.angle_max - scan.angle_min) / (fake_laser_beam_count_ - 1);
  scan.range_max = range_max_;
  scan.header = msg->header;
  scan.header.frame_id = base_laser_frame_;
  ray_caster_.laserScanCast(*msg, scan);
  
  scan.angle_min += theta;
  scan.angle_max += theta;
  manageTwist(scan);
}

} // namespace nj_oa_costmap
} // namespace lama


