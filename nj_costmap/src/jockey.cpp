#include <nj_costmap/jockey.h>

namespace nj_costmap {

Jockey::Jockey(const std::string& name, const double frontier_width) :
  lama_jockeys::NavigatingJockey(name),
  odom_frame_("odom"),
  has_crossing_(false),
  crossing_detector_(frontier_width),
  obstacle_avoider_(frontier_width / 2, "")
{
  private_nh_.getParam("odom_frame", odom_frame_);

  std::string laser_frame = "base_laser_link";
  private_nh_.getParam("laser_frame", laser_frame);
  obstacle_avoider_.laser_frame = laser_frame;

  double robot_radius;
  if (private_nh_.getParam("robot_radius", robot_radius))
  {
    obstacle_avoider_.robot_radius = robot_radius;
  }

  pub_twist_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pub_crossing_marker_ = private_nh_.advertise<visualization_msgs::Marker>("crossing_marker", 50, true);
  pub_exits_marker_ = private_nh_.advertise<visualization_msgs::Marker>("exits_marker", 50, true);
  pub_place_profile_ = private_nh_.advertise<sensor_msgs::PointCloud>("place_profile_cloud", 50, true);
  pub_crossing_ = private_nh_.advertise<lama_msgs::Crossing>("abs_crossing", 50, true);
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
      if (rel_crossing_.frontiers.size() < 3)
      {
        twist = obstacle_avoider_.getTwist(map_);
        pub_twist_.publish(twist);
        ROS_DEBUG("Obstacle avoiding");
      }
      else
      {
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

/* Callback for OccupancyGrid messages.
 */
void Jockey::handleCostmap(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map_ = *msg;
  abs_crossing_ = crossing_detector_.crossingDescriptor(map_);
 
  ROS_DEBUG("Crossing (%.3f, %.3f, %.3f), number of exits: %zu",
      abs_crossing_.center.x, abs_crossing_.center.y, abs_crossing_.radius, abs_crossing_.frontiers.size());

  // Get the rotation between odom_frame_ and the map frame.
  tf::StampedTransform tr;
  try
  {
    tf_listener_.waitForTransform(odom_frame_, map_.header.frame_id,
        map_.header.stamp, ros::Duration(0.2));
    tf_listener_.lookupTransform(odom_frame_, map_.header.frame_id, 
        map_.header.stamp, tr);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  // ROS_INFO("%s, %s", odom_frame_.c_str(), map_.header.frame_id.c_str());  // DEBUG

  // Angle from LaserScan (on which the map is base) to the map.
  const double map_relative_orientation = tf::getYaw(tr.getRotation());

  // Transform the crossing with absolute angles to relative angles.
  rel_crossing_ = abs_crossing_;
  lama_common::rotateCrossing(rel_crossing_, map_relative_orientation);
  has_crossing_ = true;

  for (size_t i = 0; i < rel_crossing_.frontiers.size(); ++i)
    ROS_DEBUG("Relative frontier angle = %.3f", rel_crossing_.frontiers[i].angle);

  // Visualization: a sphere at detected crossing center
  if (pub_crossing_marker_.getNumSubscribers())
  {
    visualization_msgs::Marker m = lama_common::getCrossingCenterMarker(map_.header.frame_id, abs_crossing_);
    pub_crossing_marker_.publish(m);
  }

  // Visualization: a line at each detected road
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

