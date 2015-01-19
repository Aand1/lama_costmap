/*
 * Obstacle avoidance with OccupancyGrid (local map).
 *
 * The local map has a fixed position relative to the robot but its
 * orientation is constant in the world reference frame.
 *
 * Drive the robot while avoiding obstacles:
 * - onTraverse and onContinue: go more or less forward depending
 *     on obstacle position. The action never stops by itself.
 *
 * Interaction with the map (created by this jockey):
 * - none.
 *
 * Interaction with the map (created by other jockeys):
 * - none.
 *
 * Subscribers (other than map-related):
 * - message type, topic default name, description
 * - nav_msgs/OccupancyGrid, "~local_map", local map with
 *   fixed position relative to the robot and fixed orientation relative
 *   to the world reference frame.
 *
 * Publishers (other than map-related):
 * - message type, topic default name, description
 * - geometry_msgs/Twist, "~cmd_vel", set velocity.
 *
 * Services used (other than map-related):
 * - none.
 *
 * Parameters:
 * - ~robot_radius, Float, NO_DEFAULT, robot radius (m).
 * - ~min_distance, Float, 2 * robot_radius, if an obstacle is closer than this,
 *     turn and don't go forward (m).
 * - ~long_distance, Float, 5 * robot_radius, if no obstacle within this
 *     distance, go straight (m).
 * - ~turnrate_collide, Float, 0.4, turn rate when obstacle closer than
 *     min_distance_ (rad/s).
 * - ~max_vel, Float, 1.0, linear velocity without obstacle (m/s).
 * - ~vel_close_obstacle, Float, 0.5, linear velocity if obstacle between
 *     min_distance and long_distance (m/s).
 * - ~turnrate_factor, Float, 0.9, if obstacle closer than long_distance
 *     turnrate = -turnrate_factor_ * mean(lateral_position_of_obstacle)
 *     (rad.m^-1.s^-1).
 * - ~laser_frame, String, "base_laser_link", frame_id of the LaserScan
 *     messages that are used to build the local map.
 */

#ifndef NJ_OA_COSTMAP_JOCKEY_H
#define NJ_OA_COSTMAP_JOCKEY_H

#include <string>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

#include <lama_jockeys/navigating_jockey.h>
#include <nj_oa_laser/jockey.h>

#include <nj_oa_costmap/twist_handler.h>

namespace nj_oa_costmap {

class Jockey : public nj_oa_laser::Jockey
{
  public :

    Jockey(const std::string& name, const double robot_radius);

    virtual void onTraverse();

  private :

    void handleMap(const nav_msgs::OccupancyGridConstPtr& msg);

    // Parameters shown outside.
    std::string base_laser_frame_;

    // Hard-coded parameters.
    const static double fake_laser_beam_count_; //!> beam count for the scan obtained from the map.
    const static double range_max_;  //!> (m), max range for beams that don't
                                     //!> encounter any occupied or unknown point on the map.

    // Internals.
    TwistHandler twist_handler_;
};

} // namespace nj_oa_costmap

#endif // NJ_OA_COSTMAP_JOCKEY_H
