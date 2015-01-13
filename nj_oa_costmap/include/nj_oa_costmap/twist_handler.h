#ifndef NJ_OA_COSTMAP_TWIST_HANDLER_H
#define NJ_OA_COSTMAP_TWIST_HANDLER_H

#include <cmath>
#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

#include <map_ray_caster/map_ray_caster.h>
#include <nj_oa_laser/twist_handler.h>

namespace nj_oa_costmap {

class TwistHandler : public nj_oa_laser::TwistHandler
{
  public :
    
    TwistHandler(const double robot_radius, const std::string& laser_frame);

    geometry_msgs::Twist getTwist(const nav_msgs::OccupancyGrid& map);

    std::string laser_frame;  //!> Name of the frame the map is based on.
    double fake_laser_beam_count; //!> beam count for the scan obtained from the map. Should be at least 2.
    double range_max;  //!> (m), max range for beams that don't encounter any occupied or unknown point on the map.

  private:

    map_ray_caster::MapRayCaster ray_caster_; //!> Ray casting with cache to compute a fake LaserScan.
    tf::TransformListener tf_listerner_;
};

} // namespace nj_oa_costmap

#endif // NJ_OA_COSTMAP_TWIST_HANDLER_H
