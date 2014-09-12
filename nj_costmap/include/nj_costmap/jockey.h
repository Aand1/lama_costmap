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
 * - sensor_msgs/LaserScan, "~/fake_scan", 360-deg laser-scan (TODO: absolute or relative orientation?).
 * - lama_msgs/Crossing, "~/abs_crossing", Crossing with absolute frontier angles.
 *
 * Services used (other than map-related):
 * - none
 *
 * Parameters:
 * - odom_frame, String, "~/odom", this is the LaserScan frame from which the map relative orientation is computed.
 */

#ifndef _NJ_COSTMAP_JOCKEY_H_
#define _NJ_COSTMAP_JOCKEY_H_

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>  // for getYaw()
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include <lama_jockeys/navigating_jockey.h>
#include <lama_msgs/Crossing.h>
#include <lama_msgs/crossing_utils.h>
#include <lama_msgs/crossing_visualization.h>
#include <goto_crossing/crossing_goer.h>

#include <nj_costmap/crossing_detector.h>
#include <nj_costmap/visualization.h>

namespace lama {
namespace nj_costmap {

class Jockey : public lama::NavigatingJockey
{
  public:

    Jockey(const std::string& name, const double frontier_width);

    virtual void onTraverse();
    virtual void onStop();
    virtual void onInterrupt();
    virtual void onContinue();

    void handleCostmap(const nav_msgs::OccupancyGridConstPtr& msg);

  private:

    ros::Publisher pub_crossing_marker_;
    ros::Publisher pub_exits_marker_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_fake_laser_;
    ros::Publisher pub_crossing_;
    
    ros::Subscriber costmap_handler_;

    std::string odom_frame_;
    double map_relative_orientation_;  //!> angle from LaserScan (on which the map is base) to the map.
    lama_msgs::Crossing abs_crossing_;  //!> Crossing descriptor with relative position and absolute angle.
    lama_msgs::Crossing rel_crossing_;  //!> Crossing descriptor with relative position and relative angle.

    lama::nj_costmap::CrossingDetector crossing_detector_;
    goto_crossing::CrossingGoer crossing_goer_;
};

} // namespace nj_costmap
} // namespace lama

#endif // _NJ_COSTMAP_JOCKEY_H_


