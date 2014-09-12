#include <nj_costmap/jockey.h>

namespace lama {
namespace nj_costmap {

Jockey::Jockey(const std::string& name, const double frontier_width) :
  lama::NavigatingJockey(name),
  crossing_detector_(frontier_width)
{
  if (!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";

  pub_twist_ = private_nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	pub_crossing_marker_ = private_nh_.advertise<visualization_msgs::Marker>("crossing_marker", 50, true);
  pub_exits_marker_ = private_nh_.advertise<visualization_msgs::Marker> ("exits_marker", 50, true);
  pub_fake_laser_ = private_nh_.advertise<sensor_msgs::LaserScan>("fake_scan", 50, true);
  pub_crossing_ = private_nh_.advertise<lama_msgs::Crossing>("abs_crossing", 50, true);
}

void Jockey::onTraverse()
{
  ROS_DEBUG("%s: Received action TRAVERSE", ros::this_node::getName().c_str());
  crossing_goer_.resetIntegrals();
  costmap_handler_ = private_nh_.subscribe("local_costmap", 1, &Jockey::handleCostmap, this);
  ROS_DEBUG("Costmap handler started");
  
  ros::Rate r(50);
  while (true)
  {
    if (server_.isPreemptRequested() && !ros::ok())
    {
      ROS_INFO("%s: Preempted", jockey_name_.c_str());
      // set the action state to preempted
      server_.setPreempted();
      break;
    }

    geometry_msgs::Twist twist;
    bool goal_reached = crossing_goer_.goto_crossing(rel_crossing_, twist);
    pub_twist_.publish(twist);
    ROS_DEBUG("twist (%.3f, %.3f)", twist.linear.x, twist.angular.z);
    //pub_twist_.publish(geometry_msgs::Twist());

    if (goal_reached)
    {
      result_.final_state = result_.DONE;
      result_.completion_time = getCompletionDuration();
      server_.setSucceeded(result_);
    }
    ros::spinOnce();
    r.sleep();
  }
  ROS_DEBUG("Exiting onTraverse");
}

void Jockey::onStop()
{
  ROS_DEBUG("%s: Received action STOP", ros::this_node::getName().c_str());
  costmap_handler_.shutdown();
  result_.final_state = result_.DONE;
  result_.completion_time = ros::Duration(0.0);
  server_.setSucceeded(result_);
}

void Jockey::onInterrupt()
{
  onStop();
}

void Jockey::onContinue()
{
  onTraverse();
}

void Jockey::handleCostmap(const nav_msgs::OccupancyGridConstPtr& msg)
{
  abs_crossing_ = crossing_detector_.detectCrossing(*msg);
 
  ROS_DEBUG("%s: crossing (%.3f, %.3f, %.3f), number of exits: %zu", ros::this_node::getName().c_str(),
        abs_crossing_.center.x, abs_crossing_.center.y, abs_crossing_.radius, abs_crossing_.frontiers.size());

  // Get the rotation between odom_frame_ and the map frame.
  tf::TransformListener tfListener;
  tf::StampedTransform tr;
  try
  {
    tfListener.waitForTransform(odom_frame_, msg->header.frame_id,
        msg->header.stamp, ros::Duration(1.0));
    tfListener.lookupTransform(odom_frame_, msg->header.frame_id, 
        msg->header.stamp, tr);
  }

  catch (tf::TransformException ex)
  {
    ROS_ERROR("Error in nj_costmap");
    ROS_ERROR("%s", ex.what());
  }
  map_relative_orientation_ = tf::getYaw(tr.getRotation());

  // Transform the crossing with absolute angles to relative angles.
  rel_crossing_ = abs_crossing_;
  rotateCrossing(map_relative_orientation_, rel_crossing_);

  for (size_t i = 0; i < rel_crossing_.frontiers.size(); ++i)
    ROS_DEBUG("Frontier angle = %.3f", rel_crossing_.frontiers[i].angle);

  // Visualization: a sphere at detected crossing center
  if (pub_crossing_marker_.getNumSubscribers())
  {
    visualization_msgs::Marker m = getCrossingCenterMarker(msg->header.frame_id, abs_crossing_);
    pub_crossing_marker_.publish(m);
  }

  // Visualization: a line at each detected road
  if (pub_exits_marker_.getNumSubscribers())
  {
    visualization_msgs::Marker m = getFrontiersMarker(msg->header.frame_id, abs_crossing_);
    pub_exits_marker_.publish(m);
  }

  // Fake laser.
  if (pub_fake_laser_.getNumSubscribers())
  {
    sensor_msgs::LaserScan scan = getFakeLaser(*msg);
    pub_fake_laser_.publish(scan);
  }

  pub_crossing_.publish(crossing_detector_.getCrossingDescriptor());
}

} // namespace nj_costmap
} // namespace lama

