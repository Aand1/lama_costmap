#ifndef _NJ_COSTMAP_CROSSDETECT_H_
#define _NJ_COSTMAP_CROSSDETECT_H_

#include <nav_msgs/OccupancyGrid.h>

#include "Voronoi.h"
#include "nj_costmap/lmap_loc_types.h"

#define FAKE_SCAN_START_ANGLE (-M_PI)
#define FAKE_SCAN_RESOLUTION  0.017453293 // (rad), 1 deg
#define OCCUPIED_THRESHOLD 40 // 0 = free, 100 = occupied

namespace Lama {
namespace Laloc {

void getXingDesc(
		const nav_msgs::OccupancyGrid& map, const double dt, const double maxFrontierAngle,
		double& cx, double& cy, double& radius, std::vector<SFrontier>& frontiers);


std::vector<double> mapToScan(const nav_msgs::OccupancyGrid& map);

} // namespace Laloc
} // namespace Lama

#endif // #ifndef _NJ_COSTMAP_CROSSDETECT_H_
