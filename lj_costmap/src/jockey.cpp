#include <lj_costmap/jockey.h>

namespace lj_costmap
{

Jockey::Jockey(std::string name, double frontier_width, double max_frontier_angle) :
  LocalizingJockey(name),
  range_cutoff_(0),
  data_received_(false),
  place_profile_interface_name_(name + "_place_profile"),
  crossing_interface_name_(name + "_crossing"),
  localize_service_("localize_in_vertex"),
  dissimilarity_server_name_("compute_dissimilarity"),
  crossing_detector_(frontier_width, max_frontier_angle)
{
  private_nh_.getParam("place_profile_interface_name", place_profile_interface_name_);
  private_nh_.getParam("crossing_interface_name", crossing_interface_name_);
  private_nh_.getParam("localize_service", localize_service_);
  private_nh_.getParam("dissimilarity_server_name", dissimilarity_server_name_);
  range_cutoff_set_ = private_nh_.getParam("range_cutoff", range_cutoff_);

  initMapPlaceProfileInterface();
  initMapCrossingInterface();
}

/** Create the getter and setter services for PlaceProfile descriptors.
 */
void Jockey::initMapPlaceProfileInterface()
{
  ros::ServiceClient client = nh_.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  while (ros::ok() && !client.waitForExistence(ros::Duration(5.0)))
  {
    ROS_WARN("Waiting for service /interface_factory");
  }
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = place_profile_interface_name_;
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::SERIALIZED;
  srv.request.get_service_message = "lama_msgs/GetPlaceProfile";
  srv.request.set_service_message = "lama_msgs/SetPlaceProfile";
  if (!client.call(srv))
  {
    ROS_ERROR_STREAM("Failed to create the LaMa interface " << place_profile_interface_name_);
    return;
  }
  // Initialize the clients for the getter and setter services (interface to map).
  place_profile_getter_ = nh_.serviceClient<lama_msgs::GetPlaceProfile>(srv.response.get_service_name);
  place_profile_setter_ = nh_.serviceClient<lama_msgs::SetPlaceProfile>(srv.response.set_service_name);
}

/** Create the setter services for Crossing descriptors.
 */
void Jockey::initMapCrossingInterface()
{
  ros::ServiceClient client = nh_.serviceClient<lama_interfaces::AddInterface>("interface_factory");
  while (ros::ok() && !client.waitForExistence(ros::Duration(5.0)))
  {
    ROS_WARN("Waiting for service /interface_factory");
  }
  lama_interfaces::AddInterface srv;
  srv.request.interface_name = crossing_interface_name_;
  srv.request.interface_type = lama_interfaces::AddInterfaceRequest::CLEARTEXT;
  srv.request.get_service_message = "lama_msgs/GetCrossing";
  srv.request.set_service_message = "lama_msgs/SetCrossing";
  if (!client.call(srv))
  {
    ROS_ERROR("Failed to create the LaMa interface %s", crossing_interface_name_.c_str());
  }
  // Initialize the client for the setter service (interface to map).
  crossing_setter_ = nh_.serviceClient<lama_msgs::SetCrossing>(srv.response.set_service_name);
}

/** Start the subscriber, wait for an OccupancyGrid and exit upon reception.
 */
void Jockey::getLiveData()
{
  ros::Subscriber costmap_handler;
  costmap_handler = private_nh_.subscribe<nav_msgs::OccupancyGrid>("local_costmap", 1, &Jockey::handleMap, this);

  /* Wait a bit to avoid the first throttled message. */
  // TODO: simplify with ROS_INFO_STREAM_DELAYED_THROTTLE, when available.
  ros::Duration(0.3).sleep();
  ros::spinOnce();
  ros::Rate r(100);
  while (ros::ok())
  {
    ros::spinOnce();
    if (data_received_)
    {
      data_received_ = false;
      break;
    }
    ROS_INFO_STREAM_THROTTLE(5, "Did not received any map on " << costmap_handler.getTopic());
    r.sleep();
  }
}

/** Callback for OccupancyGrid subscriber, receive a message and store it.
 */
void Jockey::handleMap(const nav_msgs::OccupancyGridConstPtr& msg)
{
  map_ = *msg;
  profile_ = lama_common::costmapToPlaceProfile(map_);
  data_received_ = true;
}

/** Return the vertex descriptors associated with the current robot position through result_.
 *
 * The descriptors are a LaserScan and a Crossing.
 */
// TODO: Discuss with Karel the exact role of onGetVertexDescriptor
// TODO: in particular: should it save something in the database? This jockey
// is not a learning jockey.
void Jockey::onGetVertexDescriptor()
{
  if (server_.isPreemptRequested() && !ros::ok())
  {
    ROS_INFO("%s: Preempted", jockey_name_.c_str());
    // set the action state to preempted
    server_.setPreempted();
    return;
  }

  getLiveData();

  // Add the PlaceProfile to the descriptor list.
  lama_msgs::SetPlaceProfile profile_setter_srv;
  profile_setter_srv.request.descriptor = profile_;
  if (!place_profile_setter_.call(profile_setter_srv))
  {
    ROS_ERROR("Failed to add PlaceProfile to the map");
    server_.setAborted();
    return;
  }
  ROS_DEBUG("Added PlaceProfile with id %d to the map", profile_setter_srv.response.id);
  result_.descriptor_links.push_back(placeProfileDescriptorLink(profile_setter_srv.response.id));

  // Add the Crossing to the descriptor list.
  lama_msgs::SetCrossing crossing_setter_srv;
  lama_msgs::Crossing crossing;
  if (range_cutoff_set_)
  {
    crossing = crossing_detector_.crossingDescriptor(map_, range_cutoff_);
  }
  else
  {
    crossing = crossing_detector_.crossingDescriptor(map_);
  }
  crossing_setter_srv.request.descriptor = crossing;
  if (!crossing_setter_.call(crossing_setter_srv))
  {
    ROS_DEBUG("Failed to add Crossing to the map");
    server_.setAborted();
    return;
  }
  ROS_DEBUG("Added Crossing with id %d to the map", crossing_setter_srv.response.id);
  result_.descriptor_links.push_back(crossingDescriptorLink(crossing_setter_srv.response.id));
  
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = getCompletionDuration();
  server_.setSucceeded(result_);
}

/** Return the transformation between the current and requested PlaceProfiles.
 */
void Jockey::onLocalizeInVertex()
{
  getLiveData();

  // Get the requested PlaceProfile from database.
  if (goal_.descriptor_link.interface_name != place_profile_interface_name_)
  {
    ROS_ERROR_STREAM("Expected a descriptor_link with interface " <<
        place_profile_interface_name_ << " got " <<
        goal_.descriptor_link.interface_name);
    server_.setAborted();
    return;
  }
  lama_msgs::GetPlaceProfile profile_srv;
  profile_srv.request.id = goal_.descriptor_link.descriptor_id;
  if (!place_profile_getter_.call(profile_srv))
  {
    ROS_ERROR_STREAM(jockey_name_ << ": failed to call service \"" <<
        place_profile_interface_name_ << "\"");
    server_.setAborted();
    return;
  }
  
  // Initialize the client for the localize_in_vertex service.
  ros::ServiceClient localize_client;
  localize_client = nh_.serviceClient<place_matcher_msgs::PolygonDissimilarity>(localize_service_);
  while (ros::ok() && !localize_client.waitForExistence(ros::Duration(5.0)))
  {
    ROS_WARN_STREAM("Waiting for service " << localize_client.getService());
  }

  // Compare both polygons by calling the localize service.
  result_.idata.clear();
  result_.fdata.clear();
  result_.fdata.reserve(7);  // x, y, z, qx, qy, qz, qw.
  place_matcher_msgs::PolygonDissimilarity dissimi_srv;
  dissimi_srv.request.polygon1 = profile_srv.response.descriptor.polygon;
  dissimi_srv.request.polygon2 = profile_.polygon;
  if (!localize_client.call(dissimi_srv))
  {
    ROS_ERROR_STREAM(jockey_name_ << ": failed to call service \"" << 
        localize_client.getService() << "\"");
    server_.setAborted();
    return;
  }
  result_.fdata.push_back(dissimi_srv.response.pose.position.x);
  result_.fdata.push_back(dissimi_srv.response.pose.position.y);
  result_.fdata.push_back(dissimi_srv.response.pose.position.z);
  result_.fdata.push_back(dissimi_srv.response.pose.orientation.x);
  result_.fdata.push_back(dissimi_srv.response.pose.orientation.y);
  result_.fdata.push_back(dissimi_srv.response.pose.orientation.z);
  result_.fdata.push_back(dissimi_srv.response.pose.orientation.w);

  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = getCompletionDuration();
  server_.setSucceeded(result_);
}

void Jockey::onGetDissimilarity()
{
  getLiveData();

  // Initialize the client for the dissimilarity server.
  ros::ServiceClient dissimilarity_client;
  dissimilarity_client = nh_.serviceClient<place_matcher_msgs::PolygonDissimilarity>(dissimilarity_server_name_);
  while (ros::ok() && !dissimilarity_client.waitForExistence(ros::Duration(5.0)))
  {
    ROS_WARN_STREAM("Waiting for service " << dissimilarity_client.getService());
  }

  // Get all PlaceProfile from database.
  lama_interfaces::ActOnMap srv;
  srv.request.action = lama_interfaces::ActOnMapRequest::GET_VERTEX_LIST;
  if (!map_agent_.call(srv))
  {
    ROS_ERROR_STREAM("Failed to call map agent \"" << map_agent_.getService() << "\"");
    server_.setAborted();
    return;
  }
  
  // Iterate over vertices and get the associated Polygon (from the PlaceProfile).
  std::vector<int32_t> vertices;
  vertices.reserve(srv.response.objects.size());
  std::vector<geometry_msgs::Polygon> polygons;
  polygons.reserve(srv.response.objects.size());
  for (size_t i = 0; i < srv.response.objects.size(); ++i)
  {
    // Get all PlaceProfile descriptors associated with the current vertex.
    lama_interfaces::ActOnMap desc_srv;
    desc_srv.request.action = lama_interfaces::ActOnMapRequest::GET_DESCRIPTOR_LINKS;
    desc_srv.request.object.id = srv.response.objects[i].id;
    desc_srv.request.interface_name = place_profile_interface_name_;
    if (!map_agent_.call(desc_srv))
    {
      ROS_ERROR_STREAM("Failed to call map agent \"" << map_agent_.getService() << "\"");
      server_.setAborted();
      return;
    }
    if (desc_srv.response.descriptor_links.empty())
    {
      continue;
    }
    if (desc_srv.response.descriptor_links.size() > 1)
    {
      ROS_WARN_STREAM("More than one descriptor with interface " <<
          place_profile_interface_name_ << " for vertex " <<
          desc_srv.request.object.id << ", taking the first one");
    }
    // Get the first linked PlaceProfile.
    lama_msgs::GetPlaceProfile profile_srv;
    profile_srv.request.id = desc_srv.response.descriptor_links[0].descriptor_id;
    if (!place_profile_getter_.call(profile_srv))
    {
      ROS_ERROR_STREAM(jockey_name_ << ": failed to call service \"" <<
          place_profile_interface_name_ << "\"");
      server_.setAborted();
      return;
    }
    vertices.push_back(desc_srv.request.object.id);
    polygons.push_back(profile_srv.response.descriptor.polygon);
  }
  
  // Compare them to the current polygon by calling one of the polygon_matcher service.
  place_matcher_msgs::PolygonDissimilarity dissimi_srv;
  dissimi_srv.request.polygon1 = profile_.polygon;
  result_.idata.clear();
  result_.fdata.clear();
  result_.idata.reserve(vertices.size());
  result_.fdata.reserve(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    dissimi_srv.request.polygon2 = polygons[i];
    if (!dissimilarity_client.call(dissimi_srv))
    {
      ROS_ERROR_STREAM(jockey_name_ << ": failed to call service \"" << 
          dissimilarity_client.getService() << "\"");
      server_.setAborted();
      return;
    }
    result_.idata.push_back(vertices[i]);
    result_.fdata.push_back(dissimi_srv.response.raw_dissimilarity);
  }

  ROS_INFO_STREAM(jockey_name_ << ": computed " << result_.idata.size() << " dissimilarities");
  result_.state = lama_jockeys::LocalizeResult::DONE;
  result_.completion_time = getCompletionDuration();
  server_.setSucceeded(result_);
}

/** Return a DescriptorLink for the PlaceProfile interface
 */
lama_msgs::DescriptorLink Jockey::placeProfileDescriptorLink(int32_t id)
{
  lama_msgs::DescriptorLink descriptor_link;
  descriptor_link.descriptor_id = id;
  descriptor_link.interface_name = place_profile_interface_name_;
  return descriptor_link;
}

/** Return a DescriptorLink for the Crossing interface
 */
lama_msgs::DescriptorLink Jockey::crossingDescriptorLink(int32_t id)
{
  lama_msgs::DescriptorLink descriptor_link;
  descriptor_link.descriptor_id = id;
  descriptor_link.interface_name = crossing_interface_name_;
  return descriptor_link;
}

} // namespace lj_costmap
