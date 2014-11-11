/*
 * Navigating jockey from local map
 *
 * The local map is centered on the laser scanner but its orientation is fixed.
 *
 * The role of this jockey is to travel to the next crossing.
 * The action is done when the robot reaches the crossing center.
 * It is considered to be memory-less because it uses only an internal memory
 * (in form of a local map) and do not interact with the large map.
 *
 * Interaction with the map (created by this jockey):
 * - none
 *
 * Interaction with the map (created by other jockeys):
 * - none
 *
 * Subscribers (other than map-related):
 * - nav_msgs/OccupancyGrid, "~/local_costmap", local cost map which orientation is global
 *
 * Publishers (other than map-related):
 * - geometry_msgs/Twist, "~/cmd_vel", robot set velocity
 * - visualization_msgs/Marker, "~/crossing_marker", a sphere at the crossing center.
 * - visualization_msgs/Marker, "~/exits_marker", lines from crossing center towards exits.
 * - sensor_msgs/PointCloud, "~/place_profile", point clound representing the place profile.
 * - lama_msgs/Crossing, "~/abs_crossing", Crossing with absolute frontier angles.
 *
 * Services used (other than map-related):
 * - none
 *
 * Parameters (read from this jockey):
 * - odom_frame, String, "/odom", this is the LaserScan frame from which the
 *     map relative orientation is computed.
 * - ~robot_radius, Float, frontier_width/2, robot radius (frontier_width is
 *     a constructor parameter)
 * - ~laser_frame, String, "/base_laser_link", name of the LaserScan frame the
 *     map is based on.
 */

#ifndef _NJ_COSTMAP_JOCKEY_H_
#define _NJ_COSTMAP_JOCKEY_H_

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>  // for getYaw()
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>

#include <goto_crossing/crossing_goer.h>
#include <crossing_detector/costmap_crossing_detector.h>
#include <lama_jockeys/navigating_jockey.h>
#include <lama_msgs/Crossing.h>
#include <lama_msgs/crossing_utils.h>
#include <lama_msgs/crossing_visualization.h>
#include <lama_msgs/place_profile_conversions.h>
#include <nj_oa_costmap/twist_handler.h>

namespace lama {
namespace nj_costmap {

class Jockey : public NavigatingJockey
{
  public:

    Jockey(const std::string& name, const double frontier_width);

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

    void handleCostmap(const nav_msgs::OccupancyGridConstPtr& msg);

  private:

    // Publishers and subscribers.
    ros::Publisher pub_crossing_marker_;
    ros::Publisher pub_exits_marker_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_place_profile_;
    ros::Publisher pub_crossing_;
    
    ros::Subscriber costmap_handler_;

    // Internals.
    std::string odom_frame_;
    double map_relative_orientation_;  //!> angle from LaserScan (on which the map is base) to the map.
    bool has_crossing_;  //!> true when a new crossing was computed.
    nav_msgs::OccupancyGrid map_;  //!> Last received map.
    lama_msgs::Crossing abs_crossing_;  //!> Crossing descriptor with relative position and absolute angle.
    lama_msgs::Crossing rel_crossing_;  //!> Crossing descriptor with relative position and relative angle.

    tf::TransformListener tf_listener_;
    crossing_detector::CostmapCrossingDetector crossing_detector_;  //!> Compute the crossing.
    goto_crossing::CrossingGoer crossing_goer_;  //!> Compute the twist to go to crossing center.
    nj_oa_costmap::TwistHandler obstacle_avoider_;  //!> Compute the twist for obstacle avoidance.
};

} // namespace nj_costmap
} // namespace lama

#endif // _NJ_COSTMAP_JOCKEY_H_


