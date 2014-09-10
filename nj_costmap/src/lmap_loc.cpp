#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "nj_costmap/lmap_loc.h"
#include "nj_costmap/crossDetect.h"

namespace lama {
namespace nj_costmap {

LMapLoc::LMapLoc() :
	m_cx(0.0),
	m_cy(0.0),
	m_cr(0.0)
{
}

void LMapLoc::crossDetect(nav_msgs::OccupancyGrid map)
{

	// Get the crossing center.
	getXingDesc(map, m_frontier_width, 45 * M_PI / 180.0, m_cx, m_cy, m_cr, m_frontiers);

	m_crossDescriptor.clear();
	m_crossDescriptor.push_back(m_cx);
	m_crossDescriptor.push_back(m_cy);
	m_crossDescriptor.push_back(m_cr);
	for(auto frontier : m_frontiers)
	{
		m_crossDescriptor.push_back(frontier.angle);
	}
}

std::vector<double> LMapLoc::getCrossDescriptor() const
{
	return m_crossDescriptor;
}

double LMapLoc::getCrossCenterX() const
{
	return m_cx;
}

double LMapLoc::getCrossCenterY() const
{
	return m_cy;
}

double LMapLoc::getCrossRadius() const
{
	return m_cr;
}

std::vector<double> LMapLoc::getExitAngles() const
{
	std::vector<double> exitAngles;
	for (size_t i = 3; i < m_crossDescriptor.size(); ++i)
	{
		exitAngles.push_back(m_crossDescriptor[i]);
	}
	return exitAngles;
}

void LMapLoc::setFrontierWidth(double frontier_width)
{
	m_frontier_width = frontier_width;
}

} // namespace nj_costmap
} // namespace lama
