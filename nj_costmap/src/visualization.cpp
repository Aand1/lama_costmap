/* Functions for visualization of crossings in rviz */

#include <nj_costmap/visualization.h>

namespace lama {
namespace nj_costmap {

sensor_msgs::LaserScan getFakeLaser(const nav_msgs::OccupancyGrid& msg)
{
	sensor_msgs::LaserScan scan;

	std::vector<double> scanPts = lama::nj_costmap::mapToScan(msg);
	static uint32_t seq = 0;
	scan.header.seq = seq++;
	scan.header.frame_id = msg.header.frame_id;
	scan.header.stamp = ros::Time::now();
	scan.angle_min = FAKE_SCAN_START_ANGLE;
	scan.angle_max = FAKE_SCAN_START_ANGLE + 2 * M_PI;
	scan.angle_increment = FAKE_SCAN_RESOLUTION;
	scan.scan_time = 0.005;
	scan.time_increment = scan.scan_time * FAKE_SCAN_RESOLUTION / 2 / M_PI;
	scan.range_min = 0;
	scan.range_max = 80;
	scan.ranges.assign(scanPts.begin(), scanPts.end());

	return scan;
}

} // namespace nj_costmap
} // namespace lama

