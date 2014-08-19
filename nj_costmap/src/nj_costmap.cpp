/*
 * Navigating jockey from local map
 *
 * The local map is centered on the laser scanner but its orientation is fixed.
 */

#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug
#include <actionlib/server/simple_action_server.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <lama_interfaces/NavigateAction.h>
#include <lama_interfaces/lmi_laser_descriptor_get.h>
#include <lama_interfaces/lmi_laser_descriptor_set.h>
#include <nj_costmap/lmap_loc.h>
#include <nj_costmap/visualization.h>

namespace li = lama_interfaces;

const double maxtheta = 1.6;
const double max_distance = 3.0;
const double reach_distance = 0.05;
const double min_velocity = 0.020;
const double kp_v = 0.05;  // Proportional gain for the linear velocity
const double kp_w = 0.2;  // Proportional gain for the angular velocity

typedef actionlib::SimpleActionServer<li::NavigateAction> NavServer;

ros::Publisher pub_twist;
ros::Publisher marker;
ros::Publisher roadMarker;
ros::Publisher fakeLaserMarker;
lama::Laloc::LMapLoc loc;

std::string odom_frame;

/* Return the angle from a quaternion representing a rotation around the z-axis
 *
 * The quaternion in ROS is q = (w, x, y, z), so that
 * q = (cos(a/2), ux * sin(a/2), uy * sin(a/2), uz * sin(a/2)),
 *   where a is the rotation angle and (ux, uy, uz) is the unit vector of the
 *   rotation axis.
 * For a rotation around z, we have q = (cos(a/2), 0, 0, sin(a/2)). Thus
 * a = 2 * atan2(z, w).
 */
double angleFromQuaternion(const tf::Quaternion q)
{
	if (std::fabs(q.x()) > 1e-5 || std::fabs(q.y()) > 1e-5)
	{
		ROS_WARN("Laser frame rotation is not around the z-axis, just pretending it is");
	}
	return 2 * std::atan2(q.z(), q.w());
}

/* Return the twist to reach the given goal
 *
 * xgoal[in] x position of the goal relative to map center (with orientation same as map)
 * ygoal[in] y position of the goal relative to map center (with orientation same as map)
 * map[in] map from which the orientation is extracted (the map doesn't rotate relative to the world,
 *   i.e. its rotation is the opposite of the rotation of odom_frame)
 */
geometry_msgs::Twist goToGoal(const double xgoal, const double ygoal, const nav_msgs::OccupancyGrid& map)
{
	geometry_msgs::Twist twist;
	
	// Get the rotation between the map and odom_frame.
	tf::TransformListener tfListener;
	tf::StampedTransform tr;
	try
	{
		tfListener.waitForTransform(map.header.frame_id, odom_frame,
				map.header.stamp, ros::Duration(1.0));
		tfListener.lookupTransform(map.header.frame_id, odom_frame,
				map.header.stamp, tr);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("Error in nj_costmap");
		ROS_ERROR("%s", ex.what());
	}
	
	double distance = std::sqrt(xgoal * xgoal + ygoal * ygoal);
	if (distance > max_distance)
	{
		ROS_DEBUG("distance to goal (%f) is longer than max (%f)", distance, max_distance);
	}
	// thetarobot is the rotation of the robot relative to the map.
	double thetarobot = angleFromQuaternion(tr.getRotation());
	double thetagoal = std::atan2(ygoal, xgoal); // In map coordinate system
	double dtheta = std::fmod(thetagoal - thetarobot + M_PI, 2 * M_PI) - M_PI;
	ROS_DEBUG("thetarobot: %f, thetagoal: %f, dtheta: %f", thetarobot, thetagoal, dtheta);
	if (dtheta > maxtheta) dtheta = maxtheta;
	if (dtheta < -maxtheta) dtheta = -maxtheta;

	// Only move forward if the goal is in front of the robot (+/- maxtheta)
	double vx = kp_v * distance * (maxtheta - std::fabs(dtheta)) / maxtheta; 
	double wz = kp_w * dtheta;
	if (distance < reach_distance)
	{
		vx = 0;
		wz = 0;
	}
	else if (vx < min_velocity)
	{
		vx = min_velocity;
	}

	
	twist.linear.x = vx;
	twist.angular.z = wz;
	return twist;
}

void handleCostmap(nav_msgs::OccupancyGrid msg)
{
	loc.crossDetect(msg);
	std::vector<double> desc = loc.getCrossDescriptor();

	ROS_DEBUG("CROSS x: %.3f, y: %.3f, r: %.3f, nroads: %lu", desc[0], desc[1], desc[2], desc.size() - 3);

	geometry_msgs::Twist twist = goToGoal(desc[0], desc[1], msg);
	pub_twist.publish(twist);

	// Visualization: a sphere at detected crossing center
	if (marker.getNumSubscribers())
	{
		visualization_msgs::Marker m = getXingMarker(msg.header.frame_id,
				loc.getCrossCenterX(), loc.getCrossCenterY(), loc.getCrossRadius());
		marker.publish(m);
	}

	// Visualization: a line at each detected road
	if (roadMarker.getNumSubscribers())
	{
		std::vector<double> exitAngles = loc.getExitAngles();
		visualization_msgs::Marker ml = getRoadsMarker(msg.header.frame_id, exitAngles, loc.getCrossRadius());
		roadMarker.publish(ml);
	}

	// Fake laser.
	if (fakeLaserMarker.getNumSubscribers())
	{
		sensor_msgs::LaserScan scan = getFakeLaser(msg);
		fakeLaserMarker.publish(scan);
	}
}

bool executeNavigate(const li::NavigateGoalConstPtr& goal, NavServer* server)
{
	li::lmi_laser_descriptor_get ldg;
	li::NavigateResult result;

	switch (goal->action)
	{
		case li::NavigateGoal::STOP:
			ROS_DEBUG("Navigation STOP");
			result.final_state = result.STOPPED;
			break;
		case li::NavigateGoal::TRAVERSE:
			ROS_DEBUG("Navigation START");
			ldg.request.id = goal->descriptor;
			ros::service::call("lmi_laser_descriptor_getter", ldg);
			// TODO: rewrite this file using the new Navigate.action.
			break;
	}
	return true;
}


int main(int argc, char **argv)
{
	// Debug log level
	if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
	{
		ros::console::notifyLoggerLevelsChanged();
	}

	ros::init(argc, argv, "costmap_jockey");
	ros::NodeHandle n("~");

	/* Frame of the laser sensor */
	n.param<std::string>("odom_frame", odom_frame, "odom");
	ROS_INFO("odom_frame: %s", odom_frame.c_str());

	/* Minimal frontier width */
	if (!n.hasParam("frontier_width"))
	{
		ROS_ERROR("Parameter frontier_width not set, exiting.");
		return 1;
	}
	double frontier_width;
	n.param<double>("frontier_width", frontier_width, 0.0);
	loc.setFrontierWidth(frontier_width);
	
	ros::Subscriber costmapHandler = n.subscribe<nav_msgs::OccupancyGrid> ("local_costmap", 1, handleCostmap);
	pub_twist = n.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
	marker = n.advertise<visualization_msgs::Marker> ("cross_marker",50, true);
	roadMarker = n.advertise<visualization_msgs::Marker> ("cross_line_marker", 50, true);
	fakeLaserMarker = n.advertise<sensor_msgs::LaserScan> ("fake_laser_scan", 50, true);
	
	NavServer nav_server(n, "navigating_jockey", boost::bind(&executeNavigate, _1, &nav_server), false);
	nav_server.start();

	ROS_INFO("navigating jockey with cost map started");
	ros::spin();

	return 0;
}

