#include <nj_oa_costmap/twist_handler.h>

namespace nj_oa_costmap {

TwistHandler::TwistHandler(const double robot_radius, const std::string& laser_frame) :
  nj_oa_laser::TwistHandler(robot_radius),
  laser_frame(laser_frame),
  fake_laser_beam_count(20),
  range_max(5)
{
}

geometry_msgs::Twist TwistHandler::getTwist(const nav_msgs::OccupancyGrid& map)
{
  // Get the rotation between map and the LaserScan messages that were
  // used to build the map.
  tf::StampedTransform tr;
  try
  {
    tf_listerner_.waitForTransform(laser_frame, map.header.frame_id,
        map.header.stamp, ros::Duration(0.2));
    tf_listerner_.lookupTransform(laser_frame, map.header.frame_id,
        map.header.stamp, tr);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  const double theta = tf::getYaw(tr.getRotation());

  sensor_msgs::LaserScan scan;
  scan.angle_min = -M_PI_2 - theta;
  scan.angle_max = M_PI_2 - theta;
  scan.angle_increment = (scan.angle_max - scan.angle_min) / (fake_laser_beam_count - 1);
  scan.range_max = range_max;
  scan.header = map.header;
  scan.header.frame_id = laser_frame;
  ray_caster_.laserScanCast(map, scan);
  
  scan.angle_min += theta;
  scan.angle_max += theta;

  geometry_msgs::Twist twist = nj_oa_laser::TwistHandler::getTwist(scan);
  return twist;
}

} // namespace nj_costmap
