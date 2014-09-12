#ifndef _NJ_COSTMAP_CROSSDETECT_H_
#define _NJ_COSTMAP_CROSSDETECT_H_

#include <cmath>
#include <map>
#include <cstdint>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

#include <lama_msgs/Crossing.h>
#include <lama_msgs/Frontier.h>

#include <nj_costmap/lmap_loc_types.h>
#include <nj_costmap/polygonUtils.h>
#include <Voronoi.h>
#include <ANN/ANN.h>

#define FAKE_SCAN_START_ANGLE (-M_PI)
#define FAKE_SCAN_RESOLUTION  0.017453293 // (rad), 1 deg
#define OCCUPIED_THRESHOLD 40 // 0 = free, 100 = occupied

namespace lama {
namespace nj_costmap {

using std::vector;

lama_msgs::Crossing getXingDesc(
		const nav_msgs::OccupancyGrid& map, const double dt, const double maxFrontierAngle);

vector<double> mapToScan(const nav_msgs::OccupancyGrid& map);

} // namespace nj_costmap
} // namespace lama

#endif // #ifndef _NJ_COSTMAP_CROSSDETECT_H_
