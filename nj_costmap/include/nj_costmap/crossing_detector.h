#ifndef _NJ_COSTMAP_LMAP_LOC_H_
#define _NJ_COSTMAP_LMAP_LOC_H

#include <nav_msgs/OccupancyGrid.h>
#include <lama_msgs/Crossing.h>
#include <lama_msgs/Frontier.h>

#include "nj_costmap/lmap_loc_types.h"
#include <nj_costmap/crossing_detector_helper.h>

namespace lama {
namespace nj_costmap {

class CrossingDetector
{
	public:

		CrossingDetector(const double frontier_width);

    lama_msgs::Crossing detectCrossing(const nav_msgs::OccupancyGrid& map);

    lama_msgs::Crossing getCrossingDescriptor();
		double getCrossCenterX() const;
		double getCrossCenterY() const;
		double getCrossRadius() const;
    size_t getNumExits() const;
		std::vector<double> getExitAngles() const;

		void setFrontierWidth(double frontier_width);

	private:

		double frontier_width_;  //! lines longer that this are considered frontiers

    lama_msgs::Crossing crossing_;
};

} // namespace nj_costmap
} // namespace lama

#endif // _NJ_COSTMAP_LMAP_LOC_H_
