#ifndef _NJ_COSTMAP_LMAP_LOC_H_
#define _NJ_COSTMAP_LMAP_LOC_H

#include <nav_msgs/OccupancyGrid.h>

#include "nj_costmap/lmap_loc_types.h"

namespace lama {
namespace Laloc {

class LMapLoc {
	public:
		LMapLoc();

		void crossDetect(const nav_msgs::OccupancyGrid map);

		std::vector<double> getCrossDescriptor() const;
		double getCrossCenterX() const;
		double getCrossCenterY() const;
		double getCrossRadius() const;
		std::vector<double> getExitAngles() const;

		void setFrontierWidth(double frontier_width);

	private:
		double m_frontier_width;  //! lines longer that this are considered frontiers

		double m_cx;  //<! Crossing center x-position (m)
		double m_cy;  //<! Crossing center y-position (m)
		double m_cr;  //<! Crossing radius (m)
		std::vector<SFrontier> m_frontiers;
		std::vector<double> m_crossDescriptor;

};

} // namespace Laloc
} // namespace lama

#endif // _NJ_COSTMAP_LMAP_LOC_H_
