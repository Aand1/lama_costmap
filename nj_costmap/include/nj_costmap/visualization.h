#ifndef __NJ_COSTMAP_VISUALIZATION_H__
#define __NJ_COSTMAP_VISUALIZATION_H__

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

visualization_msgs::Marker getXingMarker(const std::string& frame_id, const double x, const double y, const double radius);

visualization_msgs::Marker getRoadsMarker(const std::string& frame_id, const double x, const double y, const std::vector<double>& angles, const double length);

sensor_msgs::LaserScan getFakeLaser(nav_msgs::OccupancyGrid& msg);

#endif // #ifndef __NJ_COSTMAP_VISUALIZATION_H__
