#include <nj_costmap/crossing_detector.h>

namespace lama {
namespace nj_costmap {

CrossingDetector::CrossingDetector(const double frontier_width) :
  frontier_width_(frontier_width)
{
}

lama_msgs::Crossing CrossingDetector::detectCrossing(const nav_msgs::OccupancyGrid& map)
{
	// Get the crossing center.
  crossing_ = getXingDesc(map, frontier_width_, 45 * M_PI / 180.0);

	return crossing_;
}

lama_msgs::Crossing CrossingDetector::getCrossingDescriptor()
{
  return crossing_;
}

double CrossingDetector::getCrossCenterX() const
{
	return crossing_.center.x;
}

double CrossingDetector::getCrossCenterY() const
{
	return crossing_.center.y;
}

double CrossingDetector::getCrossRadius() const
{
	return crossing_.radius;
}

size_t CrossingDetector::getNumExits() const
{
  return crossing_.frontiers.size();
}

std::vector<double> CrossingDetector::getExitAngles() const
{
	std::vector<double> exitAngles;
	for (size_t i = 3; i < crossing_.frontiers.size(); ++i)
	{
		exitAngles.push_back(crossing_.frontiers[i].angle);
	}
	return exitAngles;
}

void CrossingDetector::setFrontierWidth(double frontier_width)
{
	frontier_width_ = frontier_width;
}

} // namespace nj_costmap
} // namespace lama
