#include <nj_costmap/jockey.h>

namespace nj_costmap
{

Jockey::Jockey(const std::string& name, const double frontier_width) :
  lama_jockeys::NavigatingJockey(name),
  odom_frame_("odom"),
  range_cutoff_(0),
  has_crossing_(false),
  last_map_orientation_(0),
  crossing_detector_(frontier_width),
  obstacle_avoider_(frontier_width / 2, "base_laser_link")
{
  private_nh_.getParam("odom_frame", odom_frame_);
  range_cutoff_set_ = private_nh_.getParam("range_cutoff", range_cutoff_);

  pub_twist_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pub_crossing_marker_ = private_nh_.advertise<visualization_msgs::Marker>("crossing_marker", 50, true);
  pub_exits_marker_ = private_nh_.advertise<visualization_msgs::Marker>("exits_marker", 50, true);
  pub_place_profile_ = private_nh_.advertise<sensor_msgs::PointCloud>("place_profile_cloud", 50, true);
  pub_crossing_ = private_nh_.advertise<lama_msgs::Crossing>("abs_crossing", 50, true);

  initTwistHandlerParam();
}

void Jockey::initTwistHandlerParam()
{
  // Parameters from nj_oa_laser::TwistHandler.
  private_nh_.getParam("robot_radius", obstacle_avoider_.robot_radius);
  private_nh_.getParam("min_distance", obstacle_avoider_.min_distance);
  private_nh_.getParam("long_distance", obstacle_avoider_.long_distance);
  private_nh_.getParam("turnrate_collide", obstacle_avoider_.turnrate_collide);
  private_nh_.getParam("vel_close_obstacle", obstacle_avoider_.vel_close_obstacle);
  private_nh_.getParam("turnrate_factor", obstacle_avoider_.turnrate_factor);
  private_nh_.getParam("max_linear_velocity", obstacle_avoider_.max_linear_velocity);
  private_nh_.getParam("max_angular_velocity", obstacle_avoider_.max_angular_velocity);

  // Parameters from nj_oa_costmap::TwistHandler.
  private_nh_.getParam("laser_frame", obstacle_avoider_.laser_frame);

  int fake_laser_beam_count;
  if (private_nh_.getParam("fake_laser_beam_count", fake_laser_beam_count))
  {
    if (fake_laser_beam_count > 1)
    {
      obstacle_avoider_.fake_laser_beam_count = fake_laser_beam_count;
    }
    else
    {
      ROS_ERROR_STREAM("Parameter " << private_nh_.getNamespace() << "/fake_laser_beam_count must be at least 2, setting to default");
    }
  }

  private_nh_.getParam("range_max", obstacle_avoider_.range_max);
}

void Jockey::onTraverse()
{
  ROS_INFO("Received action TRAVERSE or CONTINUE");
  crossing_goer_.resetIntegrals();
  costmap_handler_ = private_nh_.subscribe("local_costmap", 1, &Jockey::handleCostmap, this);
  ROS_DEBUG("Costmap handler started");
  
  ros::Rate r(100);
  while (ros::ok())
  {
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      server_.setPreempted();
      break;
    }

    if (has_crossing_)
    {
      geometry_msgs::Twist twist;
      // Use crossing_goer_ if true else obstacle_avoider_
      bool goto_crossing = true;
      if (rel_crossing_.frontiers.empty())
      {
        goto_crossing = false;
      }
      if (rel_crossing_.frontiers.size() == 1)
      {
        if ((std::abs(rel_crossing_.frontiers[0].angle) > M_PI_2))
        {
          goto_crossing = false;
        }
      }
      if (rel_crossing_.frontiers.size() == 2)
      {
        if ((std::abs(rel_crossing_.frontiers[0].angle) > M_PI_2) &&
              (std::abs(rel_crossing_.frontiers[1].angle) > M_PI_2))
        {
          goto_crossing = false;
        }
      }
      if (goto_crossing)
      {
        // Go to the crossing center if the number of exits is at least 3
        // or if one exit is in front of the robot.
        bool goal_reached = crossing_goer_.goto_crossing(rel_crossing_, twist);
        pub_twist_.publish(twist);
        ROS_DEBUG("Go to crossing");

        if (goal_reached)
        {
          result_.final_state = result_.DONE;
          result_.completion_time = getCompletionDuration();
          server_.setSucceeded(result_);
          break;
        }
      }
      else
      {
        // Go forward while avoiding obstacles.
        twist = obstacle_avoider_.getTwist(map_);
        pub_twist_.publish(twist);
        ROS_DEBUG("Avoid obstacles");
      }
      ROS_DEBUG("twist (%.3f, %.3f)", twist.linear.x, twist.angular.z);
      has_crossing_ = false;
    }
    ros::spinOnce();
    r.sleep();
  }
  ROS_DEBUG("Exiting onTraverse");
}

void Jockey::onStop()
{
  ROS_DEBUG("Received action STOP or INTERRUPT");
  costmap_handler_.shutdown();
  result_.final_state = result_.DONE;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onInterrupt()
{
  ROS_DEBUG("Received action INTERRUPT");
  onStop();
}

void Jockey::onContinue()
{
  ROS_DEBUG("Received action CONTINUE");
  onTraverse();
}

/** Callback for OccupancyGrid messages.
 */
void Jockey::handleCostmap(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map_ = *msg;
  if (range_cutoff_set_)
  {
    if (std::abs(range_cutoff_ - (-1)) < 1e-10)
    {
      // Adapt the range cut-off by first computing the radius.
      abs_crossing_ = crossing_detector_.crossingDescriptor(map_);
      ROS_DEBUG("Raw Crossing (%.3f, %.3f, %.3f), number of exits: %zu",
          abs_crossing_.center.x, abs_crossing_.center.y, abs_crossing_.radius,
          abs_crossing_.frontiers.size());

      abs_crossing_ = crossing_detector_.crossingDescriptor(map_, 1.8 * abs_crossing_.radius);
      ROS_DEBUG("Adapative range cut-off: %.3f", 1.8 * abs_crossing_.radius);
      ROS_DEBUG("Crossing with adapted range (%.3f, %.3f, %.3f), number of exits: %zu",
          abs_crossing_.center.x, abs_crossing_.center.y, abs_crossing_.radius,
          abs_crossing_.frontiers.size());
    }
    else
    {
      abs_crossing_ = crossing_detector_.crossingDescriptor(map_, range_cutoff_);
      ROS_DEBUG("Crossing: (%.3f, %.3f, %.3f), range cut-off: %.3f, number of exits: %zu",
          abs_crossing_.center.x, abs_crossing_.center.y, abs_crossing_.radius,
          range_cutoff_, abs_crossing_.frontiers.size());
    }
  }
  else
  {
    abs_crossing_ = crossing_detector_.crossingDescriptor(map_);
    ROS_DEBUG("Crossing (%.3f, %.3f, %.3f), number of exits: %zu",
        abs_crossing_.center.x, abs_crossing_.center.y, abs_crossing_.radius, abs_crossing_.frontiers.size());
  }
 
  ROS_DEBUG("Crossing (%.3f, %.3f, %.3f), number of exits: %zu",
      abs_crossing_.center.x, abs_crossing_.center.y, abs_crossing_.radius, abs_crossing_.frontiers.size());

  // Get the rotation between odom_frame_ and the map frame.
  tf::StampedTransform tr;
  bool lookup_successfull = false;
  try
  {
    tf_listener_.waitForTransform(odom_frame_, map_.header.frame_id,
        map_.header.stamp, ros::Duration(0.2));
    tf_listener_.lookupTransform(odom_frame_, map_.header.frame_id, 
        map_.header.stamp, tr);
    lookup_successfull = true;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  // Angle from LaserScan (on which the map is based) to the map.
  double map_relative_orientation = last_map_orientation_;
  if (lookup_successfull)
  {
    map_relative_orientation = tf::getYaw(tr.getRotation());
    last_map_orientation_ = map_relative_orientation;
  }

  // Transform the crossing with absolute angles to relative angles.
  rel_crossing_ = abs_crossing_;
  lama_common::rotateCrossing(rel_crossing_, map_relative_orientation);
  has_crossing_ = true;

  for (size_t i = 0; i < rel_crossing_.frontiers.size(); ++i)
    ROS_DEBUG("Relative frontier angle = %.3f", rel_crossing_.frontiers[i].angle);

  // Visualization: a sphere at detected crossing center.
  if (pub_crossing_marker_.getNumSubscribers())
  {
    visualization_msgs::Marker m = lama_common::getCrossingCenterMarker(map_.header.frame_id, abs_crossing_);
    pub_crossing_marker_.publish(m);
  }

  // Visualization: a line at each detected road.
  if (pub_exits_marker_.getNumSubscribers())
  {
    visualization_msgs::Marker m = lama_common::getFrontiersMarker(map_.header.frame_id, abs_crossing_);
    pub_exits_marker_.publish(m);
  }

  // PlaceProfile visualization message.
  if (pub_place_profile_.getNumSubscribers())
  {
    sensor_msgs::PointCloud cloud = lama_common::placeProfileToPointCloud(crossing_detector_.getPlaceProfile());
    pub_place_profile_.publish(cloud);
  }

  pub_crossing_.publish(rel_crossing_);
}

} // namespace nj_costmap

