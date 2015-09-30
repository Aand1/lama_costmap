/*
 *  * Node using a Navigating jockey from local map
 *   *
 *    * Parameters:
 *     * navigating_jockey_server_name, String, node_name + "_server"
 *      * odom_frame, String, "odom", frame of the laser sensor
 *       * frontier_width, String, no default, how wide must an exit be
 *        *
 *        */

#include <ros/ros.h>
#include <ros/console.h> // to change the log level to debug
#include <nav_msgs/OccupancyGrid.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>  // for getYaw()
#include <map_ray_caster/map_ray_caster.h>
#include <sensor_msgs/LaserScan.h>

std::string odom_frame_;
tf::TransformListener* tf_listener_;
map_ray_caster::MapRayCaster caster_;
ros::Publisher* fake_laser_publisher_;

sensor_msgs::LaserScan rotateScan(sensor_msgs::LaserScan original_scan, float rotation) {
  sensor_msgs::LaserScan scan = original_scan;
  while (rotation > 0 ) {
     rotation -= 2*M_PI;
  }
  int shift = -rotation/original_scan.angle_increment;
  for (size_t index = 0; index < original_scan.ranges.size(); index++) {
     scan.ranges[index] = original_scan.ranges[(index+shift)%original_scan.ranges.size()];
     if (scan.ranges[index] > 0.99*scan.range_max) {
        scan.ranges[index] = scan.range_max*2;
     }
  }
  return scan;

}

void handleCostmap(const nav_msgs::OccupancyGridConstPtr& msg) {
  // Get the rotation between odom_frame_ and the map frame.
  tf::StampedTransform tr;
  float map_relative_orientation = 0.0f;
  bool lookup_successfull = false;
  try
  {
    tf_listener_->waitForTransform(odom_frame_, msg->header.frame_id,
        msg->header.stamp, ros::Duration(0.2));
    tf_listener_->lookupTransform(odom_frame_, msg->header.frame_id, 
        msg->header.stamp, tr);
    lookup_successfull = true;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  if (lookup_successfull) {
    map_relative_orientation = tf::getYaw(tr.getRotation());
  }
  sensor_msgs::LaserScan scan;
  scan.range_max = 2;
  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  scan.angle_increment  = 2*M_PI/ 400;
  scan.header.frame_id = "base_laser_link";
  caster_.laserScanCast(*msg, scan);   
  fake_laser_publisher_->publish(rotateScan(scan,map_relative_orientation));
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "default_name_fake_laser");
   // Debug log level
//   if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
//   {
//      ros::console::notifyLoggerLevelsChanged();
//   }

   ros::NodeHandle nh; // Log in jockey works better with this line.
   ros::NodeHandle private_nh("~");
   odom_frame_ = "base_link";   
   ros::Subscriber costmap_handler_ = nh.subscribe("local_costmap", 1, handleCostmap);
   ros::Publisher fk = nh.advertise<sensor_msgs::LaserScan>("fake_laser",10);
   fake_laser_publisher_ = &fk;
   tf_listener_ = new tf::TransformListener();

   ros::spin();

   return 0;
}



