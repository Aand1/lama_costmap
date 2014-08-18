/* Functions for visualization of crossings in rviz */

#include <nj_costmap/visualization.h>

#include <nj_costmap/crossDetect.h> // for the fake laser

/* Return the marker for the visualization of the crossing center
 */
visualization_msgs::Marker getXingMarker(const std::string& frame_id, const double x, const double y, const double radius)
{
	visualization_msgs::Marker m;
	m.header.frame_id = frame_id;
	m.ns = "crossing_center";
	m.type = visualization_msgs::Marker::SPHERE;
	m.pose.position.x = x;
	m.pose.position.y = y;
	m.pose.position.z = 0;
	m.pose.orientation.w = 1.0;
	m.scale.x = radius;
	m.scale.y = radius;
	m.scale.z = 1;
	m.color.r = 1.0;
	m.color.g = 1.0;
	m.color.a = 0.5;
	return m;
}

/* Return the marker for the visualization of the roads
 */
visualization_msgs::Marker getRoadsMarker(const std::string& frame_id, const std::vector<double>& angles, const double length)
{
	visualization_msgs::Marker m;
	m.header.frame_id = frame_id;
	m.ns = "road_direction";
	m.type = visualization_msgs::Marker::LINE_LIST;
	m.pose.orientation.w = 1.0;
	m.scale.x = 0.1;
	m.color.r = 0.0;
	m.color.g = 0.0;
	m.color.b = 1.0;
	m.color.a = 0.5;
	for(auto angle : angles)
	{
		geometry_msgs::Point p;
		m.points.push_back(p);
		p.x = length * cos(angle);
		p.y = length * sin(angle);
		m.points.push_back(p);
	}
	return m;
}

sensor_msgs::LaserScan getFakeLaser(nav_msgs::OccupancyGrid& msg)
{
	sensor_msgs::LaserScan scan;

	std::vector<double> scanPts = Lama::Laloc::mapToScan(msg);
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

