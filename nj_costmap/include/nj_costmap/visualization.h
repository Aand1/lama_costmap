#ifndef __NJ_COSTMAP_VISUALIZATION_H__
#define __NJ_COSTMAP_VISUALIZATION_H__

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include <nj_costmap/crossing_detector_helper.h> // for the fake laser (mapToScan)

namespace lama {
namespace nj_costmap {

sensor_msgs::LaserScan getFakeLaser(const nav_msgs::OccupancyGrid& msg);

} // namespace nj_costmap
} // namespace lama

#endif // #ifndef __NJ_COSTMAP_VISUALIZATION_H__
